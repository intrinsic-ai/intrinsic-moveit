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

#include "moveit_plan_motion_skill.h"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/message.h"
#include "intrinsic/skills/cc/skill_utils.h"
#include "intrinsic/util/status/status_macros.h"

#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "moveit_plan_motion_skill.pb.h"

namespace com::generic::skills::motion {

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

class MotionPlanningRosClient : public rclcpp::Node {
 public:
  MotionPlanningRosClient() : Node("motion_planning_skill_client") {}

  absl::StatusOr<moveit_msgs::srv::GetMotionPlan::Response::SharedPtr> CallGetMotionPlan(
      const std::string& service_name,
      std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request> request,
      double timeout_ms) {
    auto client = this->create_client<moveit_msgs::srv::GetMotionPlan>(service_name);
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

std::unique_ptr<intrinsic::skills::SkillInterface> MotionPlanningSkill::CreateSkill() {
  return std::make_unique<MotionPlanningSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>> MotionPlanningSkill::Execute(
    const intrinsic::skills::ExecuteRequest& request,
    intrinsic::skills::ExecuteContext& context) {

  (void)context;
  INTR_ASSIGN_OR_RETURN(auto params, request.params<MotionPlanningParams>());

  std::string group_name = params.group_name();
  if (group_name.empty()) {
    return absl::InvalidArgumentError("group_name must not be empty.");
  }

  double timeout_ms = params.timeout_ms();
  if (timeout_ms <= 0.0) {
    return absl::InvalidArgumentError("timeout_ms must be greater than zero.");
  }

  LOG(INFO) << "Calling motion planning service for group: " << group_name;

  auto client = std::make_shared<MotionPlanningRosClient>();
  auto req = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();
  req->motion_plan_request.group_name = group_name;

  const std::string service_name = "motion_planning/get_motion_plan";
  auto call_res = client->CallGetMotionPlan(service_name, req, timeout_ms);

  if (!call_res.ok()) {
    LOG(ERROR) << "Motion planning service call failed: " << call_res.status().message();
    return call_res.status();
  }

  auto response_proto = std::make_unique<MotionPlanningResult>();

  auto response = call_res.value();
  bool success = (response->motion_plan_response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  response_proto->set_success(success);
  response_proto->set_planning_time(response->motion_plan_response.planning_time);

  if (success) {
    const auto& joint_trajectory = response->motion_plan_response.trajectory.joint_trajectory;
    for (const auto& point : joint_trajectory.points) {
      // 1. Populate geometric trajectory (positions only)
      auto* geometric_vec = response_proto->add_geometric_trajectory();
      for (double pos : point.positions) {
        geometric_vec->add_joints(pos);
      }

      // 2. Populate parameterized trajectory (JointStatePVA: position, velocity, acceleration)
      auto* parameterized_state = response_proto->add_parameterized_trajectory();
      for (double pos : point.positions) {
        parameterized_state->add_position(pos);
      }
      for (double vel : point.velocities) {
        parameterized_state->add_velocity(vel);
      }
      for (double acc : point.accelerations) {
        parameterized_state->add_acceleration(acc);
      }
    }
  }

  LOG(INFO) << "Motion planning call completed. Success: " << (success ? "yes" : "no")
            << ", Time: " << response->motion_plan_response.planning_time << "s"
            << ", Waypoints: " << response_proto->geometric_trajectory_size();

  return response_proto;
}

}  // namespace com::generic::skills::motion
