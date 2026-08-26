// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "moveit_plan_grasp_skill.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/message.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/skills/cc/skill_utils.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/transform_node.h"
#include "intrinsic/world/objects/world_object.h"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_plan_grasp_skill.pb.h"
#include "moveit_planning_interfaces/srv/plan_grasps.hpp"
#include "rclcpp/rclcpp.hpp"

namespace com::generic::skills::grasp {

std::shared_ptr<moveit_planning_interfaces::srv::PlanGrasps::Request>
CreatePlanGraspsRequest(
    const GraspPlanningParams& params,
    const intrinsic_proto::world::ObjectReference& candidate) {
  auto req =
      std::make_shared<moveit_planning_interfaces::srv::PlanGrasps::Request>();

  // Planning group: default to "ur_manipulator"
  req->group_name = (params.has_group_name() && !params.group_name().empty())
                        ? params.group_name()
                        : "ur_manipulator";

  // End effector group: default to "hand"
  req->end_effector_group =
      (params.has_end_effector_group() && !params.end_effector_group().empty())
          ? params.end_effector_group()
          : "hand";

  // Timeout: default to 10000.0 ms (10.0 s)
  double timeout_ms = (params.has_timeout_ms() && params.timeout_ms() > 0.0)
                          ? params.timeout_ms()
                          : 10000.0;
  req->planning_timeout_sec = timeout_ms / 1000.0;

  // Retract distance: default to 0.1 m
  req->retract_dist_m =
      (params.has_retract_dist_m() && params.retract_dist_m() > 0.0)
          ? params.retract_dist_m()
          : 0.1;

  // Extract box grasp annotation parameters
  if (params.has_box_grasp_annotations()) {
    const auto& box_annotations = params.box_grasp_annotations();
    for (int surface : box_annotations.surfaces()) {
      req->surfaces.push_back(surface);
    }
    req->num_rotations = box_annotations.num_rotations();

    if (box_annotations.has_obj_dims_in_meters()) {
      req->obj_dims_in_meters.x = box_annotations.obj_dims_in_meters().x();
      req->obj_dims_in_meters.y = box_annotations.obj_dims_in_meters().y();
      req->obj_dims_in_meters.z = box_annotations.obj_dims_in_meters().z();
    }

    if (box_annotations.has_obj_t_obj_center()) {
      const auto& center_pose = box_annotations.obj_t_obj_center();
      if (center_pose.has_position()) {
        req->obj_t_obj_center.position.x = center_pose.position().x();
        req->obj_t_obj_center.position.y = center_pose.position().y();
        req->obj_t_obj_center.position.z = center_pose.position().z();
      }
      if (center_pose.has_orientation()) {
        req->obj_t_obj_center.orientation.x = center_pose.orientation().x();
        req->obj_t_obj_center.orientation.y = center_pose.orientation().y();
        req->obj_t_obj_center.orientation.z = center_pose.orientation().z();
        req->obj_t_obj_center.orientation.w = center_pose.orientation().w();
      }
    }
  }

  // Extract tool_frame from TransformNodeReference if provided
  if (params.has_tool_frame()) {
    if (!params.tool_frame().id().empty()) {
      req->tool_frame = params.tool_frame().id();
    } else if (params.tool_frame().has_by_name()) {
      const auto& by_name = params.tool_frame().by_name();
      if (by_name.has_frame() && !by_name.frame().frame_name().empty()) {
        req->tool_frame = by_name.frame().frame_name();
      } else if (by_name.has_object() &&
                 !by_name.object().object_name().empty()) {
        req->tool_frame = by_name.object().object_name();
      }
    }
  }

  if (!candidate.id().empty()) {
    req->target.id = candidate.id();
  } else if (candidate.has_by_name() &&
             !candidate.by_name().object_name().empty()) {
    req->target.id = candidate.by_name().object_name();
  }

  return req;
}

namespace {
class InitRos {
 public:
  InitRos() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};
InitRos init;
using PlanGrasps = moveit_planning_interfaces::srv::PlanGrasps;

struct ObjectPlanningResult {
  intrinsic_proto::world::ObjectReference candidate_object;
  moveit_msgs::msg::Grasp grasp;
  geometry_msgs::msg::PoseStamped pre_grasp_pose;
};

class GraspPlanningRosClient : public rclcpp::Node {
 public:
  GraspPlanningRosClient() : Node("grasp_planning_skill_client") {}

  absl::StatusOr<PlanGrasps::Response::SharedPtr> CallGraspPlanning(
      const std::string& service_name,
      std::shared_ptr<PlanGrasps::Request> request, double timeout_ms) {
    auto client = this->create_client<PlanGrasps>(service_name);
    auto timeout = std::chrono::milliseconds(static_cast<int>(timeout_ms));
    if (!client->wait_for_service(timeout)) {
      return absl::UnavailableError(
          absl::StrCat("Service not reachable: ", service_name));
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result, timeout) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return absl::DeadlineExceededError(absl::StrCat(
          "Timed out waiting for service response: ", service_name));
    }
    return result.get();
  }
};
}  // namespace

std::unique_ptr<intrinsic::skills::SkillInterface>
GraspPlanningSkill::CreateSkill() {
  return std::make_unique<GraspPlanningSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
GraspPlanningSkill::Execute(const intrinsic::skills::ExecuteRequest& request,
                            intrinsic::skills::ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(auto params, request.params<GraspPlanningParams>());

  if (params.candidate_objects_size() == 0) {
    return absl::InvalidArgumentError(
        "candidate_objects must contain at least one object");
  }

  // Resolve planning parameters with fallbacks
  std::string group_name =
      (params.has_group_name() && !params.group_name().empty())
          ? params.group_name()
          : "ur_manipulator";

  double timeout_ms = (params.has_timeout_ms() && params.timeout_ms() > 0.0)
                          ? params.timeout_ms()
                          : 10000.0;

  // Resolve plan_id
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  std::string plan_id = (params.has_plan_id() && !params.plan_id().empty())
                            ? params.plan_id()
                            : ("grasp_plan_" + std::to_string(now_ms));

  // Determine max number of grasps
  int64_t max_grasps = 1;
  if (params.has_max_num_grasps() && params.max_num_grasps() > 0) {
    max_grasps = params.max_num_grasps();
  }

  LOG(INFO) << "Calling grasp planning service for group: " << group_name
            << " (plan_id: " << plan_id << ", max_grasps: " << max_grasps
            << ")";

  auto client = std::make_shared<GraspPlanningRosClient>();
  const std::string service_name = "grasp_planning/plan_grasps";

  std::vector<ObjectPlanningResult> successful_results;

  for (const auto& candidate_object : params.candidate_objects()) {
    auto req = CreatePlanGraspsRequest(params, candidate_object);

    LOG(INFO) << "Calling grasp planning service for candidate object: "
              << req->target.id;

    auto call_res = client->CallGraspPlanning(service_name, req, timeout_ms);
    if (!call_res.ok()) {
      LOG(ERROR) << "Grasp planning service call failed for object: "
                 << req->target.id << ": " << call_res.status().message();
      continue;
    }

    auto object_response = call_res.value();

    const bool object_success =
        object_response->error_code.val ==
            moveit_msgs::msg::MoveItErrorCodes::SUCCESS &&
        !object_response->grasps.empty();

    if (!object_success) {
      LOG(INFO) << "No valid grasps found for candidate object: "
                << req->target.id;
      continue;
    }

    if (object_response->pre_grasp_poses.size() !=
        object_response->grasps.size()) {
      return absl::InternalError(absl::StrCat(
          "Grasp planning service returned mismatched grasps (",
          object_response->grasps.size(), ") and pre_grasp_poses (",
          object_response->pre_grasp_poses.size(), ") for object '",
          req->target.id, "'."));
    }

    for (std::size_t grasp_index = 0;
         grasp_index < object_response->grasps.size(); ++grasp_index) {
      successful_results.push_back(
          {candidate_object, object_response->grasps[grasp_index],
           object_response->pre_grasp_poses[grasp_index]});
    }
  }

  std::sort(successful_results.begin(), successful_results.end(),
            [](const ObjectPlanningResult& first_candidate,
               const ObjectPlanningResult& second_candidate) {
              return first_candidate.grasp.grasp_quality >
                     second_candidate.grasp.grasp_quality;
            });

  auto response_proto = std::make_unique<GraspPlanningResult>();
  response_proto->set_plan_id(plan_id);

  bool success = !successful_results.empty();
  response_proto->set_success(success);

  if (success) {
    const auto& selected_result = successful_results.front();
    const auto& top_grasp = selected_result.grasp;

    // Handle updating existing frames in the World Service relative to the
    // candidate object without reparenting
    if (params.has_output_grasp_frame() || params.has_output_pregrasp_frame()) {
      auto& object_world = context.object_world();

      const auto& target_object_ref = selected_result.candidate_object;
      auto target_object_or = object_world.GetObject(target_object_ref);
      if (!target_object_or.ok()) {
        return absl::NotFoundError(absl::StrCat(
            "Candidate object could not be resolved in World Service: ",
            target_object_or.status().message()));
      }
      const auto target_object_node = target_object_or->AsTransformNode();

      intrinsic::eigenmath::Vector3d grasp_trans(
          top_grasp.grasp_pose.pose.position.x,
          top_grasp.grasp_pose.pose.position.y,
          top_grasp.grasp_pose.pose.position.z);
      intrinsic::eigenmath::Quaterniond grasp_rot(
          top_grasp.grasp_pose.pose.orientation.w,
          top_grasp.grasp_pose.pose.orientation.x,
          top_grasp.grasp_pose.pose.orientation.y,
          top_grasp.grasp_pose.pose.orientation.z);
      intrinsic::Pose3d object_t_grasp(grasp_rot, grasp_trans);

      if (params.has_output_grasp_frame()) {
        auto grasp_node_or =
            object_world.GetTransformNode(params.output_grasp_frame());
        if (!grasp_node_or.ok()) {
          return absl::NotFoundError(absl::StrCat(
              "Output grasp frame could not be resolved in World Service: ",
              grasp_node_or.status().message()));
        }
        auto status = object_world.UpdateTransform(
            target_object_node, *grasp_node_or, *grasp_node_or, object_t_grasp);
        if (!status.ok()) {
          return absl::InternalError(
              absl::StrCat("Failed to update output_grasp_frame transform: ",
                           status.message()));
        }
      }

      if (params.has_output_pregrasp_frame()) {
        auto pregrasp_node_or =
            object_world.GetTransformNode(params.output_pregrasp_frame());
        if (!pregrasp_node_or.ok()) {
          return absl::NotFoundError(absl::StrCat(
              "Output pregrasp frame could not be resolved in World Service: ",
              pregrasp_node_or.status().message()));
        }
        const auto& top_pregrasp = selected_result.pre_grasp_pose;
        intrinsic::eigenmath::Vector3d pregrasp_trans(
            top_pregrasp.pose.position.x, top_pregrasp.pose.position.y,
            top_pregrasp.pose.position.z);
        intrinsic::eigenmath::Quaterniond pregrasp_rot(
            top_pregrasp.pose.orientation.w, top_pregrasp.pose.orientation.x,
            top_pregrasp.pose.orientation.y, top_pregrasp.pose.orientation.z);
        intrinsic::Pose3d object_t_pregrasp(pregrasp_rot, pregrasp_trans);

        auto status =
            object_world.UpdateTransform(target_object_node, *pregrasp_node_or,
                                         *pregrasp_node_or, object_t_pregrasp);
        if (!status.ok()) {
          return absl::InternalError(
              absl::StrCat("Failed to update output_pregrasp_frame transform: ",
                           status.message()));
        }
      }
    }

    // Populate the result protobuf with grasps
    const size_t grasp_count =
        std::min(static_cast<size_t>(max_grasps), successful_results.size());
    for (size_t i = 0; i < grasp_count; ++i) {
      const auto& result = successful_results[i];
      const auto& grasp = result.grasp;

      auto* pose_grasp = response_proto->add_grasps();
      pose_grasp->set_grasp_id(grasp.id);

      // Set target object reference and object frame
      std::string target_name;

      *pose_grasp->mutable_object() = result.candidate_object;
      if (result.candidate_object.has_by_name()) {
        target_name = result.candidate_object.by_name().object_name();
      } else if (!result.candidate_object.id().empty()) {
        target_name = result.candidate_object.id();
      }

      if (target_name.empty()) {
        target_name = grasp.grasp_pose.header.frame_id;
        if (!target_name.empty()) {
          pose_grasp->mutable_object()->mutable_by_name()->set_object_name(
              target_name);
        }
      }

      if (!target_name.empty()) {
        pose_grasp->mutable_object_frame()
            ->mutable_by_name()
            ->mutable_object()
            ->set_object_name(target_name);
      }

      // Grasp frame mapping
      if (i == 0 && params.has_output_grasp_frame()) {
        *pose_grasp->mutable_grasp_frame() = params.output_grasp_frame();
      } else if (!target_name.empty()) {
        pose_grasp->mutable_grasp_frame()
            ->mutable_by_name()
            ->mutable_frame()
            ->set_object_name(target_name);
        pose_grasp->mutable_grasp_frame()
            ->mutable_by_name()
            ->mutable_frame()
            ->set_frame_name(grasp.id);
      }

      // Pre-grasp frame mapping
      if (i == 0 && params.has_output_pregrasp_frame()) {
        *pose_grasp->mutable_pre_grasp_frame() = params.output_pregrasp_frame();
      } else if (!target_name.empty()) {
        pose_grasp->mutable_pre_grasp_frame()
            ->mutable_by_name()
            ->mutable_frame()
            ->set_object_name(target_name);
        pose_grasp->mutable_pre_grasp_frame()
            ->mutable_by_name()
            ->mutable_frame()
            ->set_frame_name(grasp.id + "_pregrasp");
      }

      // Populate pregrasp and grasp posture / IK solutions
      if (!grasp.pre_grasp_posture.points.empty()) {
        for (double pos : grasp.pre_grasp_posture.points[0].positions) {
          pose_grasp->mutable_pregrasp_ik_solution()->add_joints(pos);
        }
      }

      if (!grasp.grasp_posture.points.empty()) {
        for (double pos : grasp.grasp_posture.points[0].positions) {
          pose_grasp->mutable_grasp_ik_solution()->add_joints(pos);
        }
      }
    }
  }

  LOG(INFO) << "Grasp planning call completed. Success: "
            << (success ? "yes" : "no")
            << ", Grasps returned: " << response_proto->grasps_size();

  return response_proto;
}

}  // namespace com::generic::skills::grasp
