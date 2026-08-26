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

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>

#include <Eigen/Geometry>
#include <atomic>
#include <boost/any.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/grasp.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_planning_interfaces/srv/plan_grasps.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>

#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "moveit_planning_service.pb.h"
#include "moveit_planning_service/generate_box_grasp_poses.hpp"
#include "moveit_planning_service/object_geometry.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

namespace mtc = moveit::task_constructor;
using PlanGrasps = moveit_planning_interfaces::srv::PlanGrasps;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;
using PlanningSceneClient = rclcpp::Client<GetPlanningScene>;
using moveit_planning_service::computeObjectBoundingBox;
using moveit_planning_service::GenerateBoxGraspPoses;
using moveit_planning_service::isometry_to_pose;
using moveit_planning_service::pose_to_isometry;
namespace {

const mtc::SolutionBase* findSubSolution(const mtc::SolutionBase* s,
                                         const mtc::Stage* target_stage) {
  if (s == nullptr || target_stage == nullptr) return nullptr;
  if (s->creator() == target_stage) {
    return s;
  }
  if (const auto* seq = dynamic_cast<const mtc::SolutionSequence*>(s)) {
    for (const auto* sub : seq->solutions()) {
      if (const auto* match = findSubSolution(sub, target_stage)) {
        return match;
      }
    }
  }
  if (const auto* wrap = dynamic_cast<const mtc::WrappedSolution*>(s)) {
    return findSubSolution(wrap->wrapped(), target_stage);
  }
  return nullptr;
}

}  // namespace

/**
 * @brief Handles motion planning requests.
 */
void handle_motion_plan_request(
    const std::shared_ptr<std::atomic<bool>>& scene_ready,
    const std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request> request,
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Response> response) {
  if (!scene_ready->load()) {
    RCLCPP_WARN(
        rclcpp::get_logger("moveit_planning_service"),
        "Rejecting motion planning request: collision scene is not ready yet.");
    response->motion_plan_response.error_code.val =
        moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("moveit_planning_service"),
              "Received motion planning request for group: '%s'",
              request->motion_plan_request.group_name.c_str());

  // Verify MTC linking
  try {
    moveit::task_constructor::Task task("verification_task");
    RCLCPP_INFO(rclcpp::get_logger("moveit_planning_service"),
                "Successfully instantiated verification MTC task: %s",
                task.name().c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("moveit_planning_service"),
                 "Failed to instantiate verification MTC task: %s", e.what());
  }

  // TODO: implement actual free-space or Cartesian motion planning, replace
  // dummy code
  response->motion_plan_response.group_name =
      request->motion_plan_request.group_name;
  response->motion_plan_response.planning_time =
      0.05;  // 50ms dummy planning time
  response->motion_plan_response.error_code.val =
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  RCLCPP_INFO(rclcpp::get_logger("moveit_planning_service"),
              "Successfully generated dummy motion plan.");
}

/**
 * @brief Resolves an incoming target object ID against existing collision
 * objects in the MoveIt planning scene.
 *
 * In Flowstate's Object World hierarchy, a WorldObject is a container of
 * entities (such as "whole"). When moveit_scene_bridge exports collision
 * objects and TF frames to ROS 2 and MoveIt, it formats the frame and collision
 * object ID as:
 *     "<object_name>/<entity_name>" (e.g. "building_block/whole")
 *
 * However, upstream Flowstate skills, high-level diagrams, and user-initiated
 * service calls typically reference the root object name ("building_block") or
 * may include a leading slash ("/building_block").
 *
 * This function resolves the target ID through the following priority order:
 * 1. Exact match: checks if `target_id` directly matches an existing collision
 * object ID.
 * 2. Leading slash stripped: checks `clean_id` without a leading '/'.
 * 3. Canonical Flowstate entity suffix: checks `<clean_id>/whole`.
 * 4. Prefix match: checks for any collision object starting with `<clean_id>/`
 * (e.g., `<clean_id>/<entity>`).
 *
 * @param existing_object_ids List of collision object IDs currently registered
 * in the planning scene.
 * @param target_id Target ID provided in the incoming request.
 * @return Resolved collision object ID, or original target_id if no matching
 * object was found.
 */
std::string resolve_target_object_id(
    const std::vector<std::string>& existing_object_ids,
    const std::string& target_id) {
  if (target_id.empty()) {
    return target_id;
  }

  // 1. Exact match
  for (const auto& id : existing_object_ids) {
    if (id == target_id) {
      return id;
    }
  }

  // 2. Strip leading slash
  std::string clean_id = target_id;
  if (!clean_id.empty() && clean_id.front() == '/') {
    clean_id.erase(0, 1);
  }
  for (const auto& id : existing_object_ids) {
    if (id == clean_id) {
      return id;
    }
  }

  // 3. Canonical Flowstate entity suffix (<clean_id>/whole)
  const std::string canonical_whole = clean_id + "/whole";
  for (const auto& id : existing_object_ids) {
    if (id == canonical_whole) {
      return id;
    }
  }

  // 4. Prefix match against any <clean_id>/<entity>
  const std::string prefix = clean_id + "/";
  for (const auto& id : existing_object_ids) {
    if (id.rfind(prefix, 0) == 0) {
      return id;
    }
  }

  return target_id;
}

/**
 * @brief Queries MoveIt's planning scene service for the list of existing world
 * collision object names.
 *
 * @param node Shared pointer to the ROS 2 node.
 * @param timeout Maximum wait time for the service response.
 * @return Vector of collision object IDs found in the planning scene.
 */
std::vector<std::string> get_planning_scene_object_ids(
    const rclcpp::Node::SharedPtr& node,
    const PlanningSceneClient::SharedPtr& get_scene_client,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) {
  std::vector<std::string> object_ids;

  if (!get_scene_client->service_is_ready()) {
    if (!get_scene_client->wait_for_service(timeout)) {
      return object_ids;
    }
  }

  auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  req->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;

  auto future = get_scene_client->async_send_request(req);
  if (future.wait_for(timeout) != std::future_status::ready) {
    get_scene_client->remove_pending_request(future);
    RCLCPP_WARN(node->get_logger(),
                "Timed out while retrieving planning scene object IDs.");
    return object_ids;
  }
  try {
    const auto resp = future.get();
    for (const auto& obj : resp->scene.world.collision_objects) {
      object_ids.push_back(obj.id);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to retrieve planning scene object IDs: %s", e.what());
  }
  return object_ids;
}

/**
 * @brief Retrieves a collision object and its geometry from the planning scene.
 *
 * @param node Shared pointer to the ROS 2 node.
 * @param object_id Exact collision object ID to retrieve.
 * @param timeout Maximum wait time for the service response.
 * @return The collision object if found, otherwise std::nullopt.
 */

std::optional<moveit_msgs::msg::CollisionObject>
get_planning_scene_collision_object(
    const rclcpp::Node::SharedPtr& node,
    const PlanningSceneClient::SharedPtr& get_scene_client,
    const std::string& object_id,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) {
  if (!get_scene_client->service_is_ready() &&
      !get_scene_client->wait_for_service(timeout)) {
    RCLCPP_WARN(
        node->get_logger(),
        "Planning scene service is unavailable while retrieving object '%s'.",
        object_id.c_str());
    return std::nullopt;
  }

  auto request =
      std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

  auto future = get_scene_client->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    get_scene_client->remove_pending_request(future);
    RCLCPP_WARN(node->get_logger(),
                "Timed out while retrieving collision object '%s'.",
                object_id.c_str());
    return std::nullopt;
  }

  try {
    const auto response = future.get();
    for (const auto& object : response->scene.world.collision_objects) {
      if (object.id == object_id) {
        return object;
      }
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to retrieve collision object '%s': %s",
                 object_id.c_str(), e.what());
    return std::nullopt;
  }

  RCLCPP_WARN(node->get_logger(),
              "Collision object '%s' was not found in the planning scene.",
              object_id.c_str());

  return std::nullopt;
}

/**
 * @brief Handles grasp planning requests.
 */
void handle_grasp_planning_request(
    const rclcpp::Node::SharedPtr& node,
    const PlanningSceneClient::SharedPtr& planning_scene_client,
    const std::shared_ptr<std::atomic<bool>>& scene_ready,
    const std::shared_ptr<PlanGrasps::Request> request,
    std::shared_ptr<PlanGrasps::Response> response) {
  if (!scene_ready->load()) {
    RCLCPP_WARN(
        node->get_logger(),
        "Rejecting grasp planning request: collision scene is not ready yet.");
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }

  if (request->target.id.empty()) {
    RCLCPP_ERROR(node->get_logger(), "target.id is empty.");
    response->error_code.val =
        moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME;
    return;
  }

  // Resolve target object ID against the active planning scene
  // (e.g. maps "building_block" or "/building_block" to "building_block/whole")
  double timeout_sec = 5.0;
  node->get_parameter("ros_service_call_timeout_sec", timeout_sec);
  const auto service_call_timeout =
      std::chrono::milliseconds(static_cast<int64_t>(timeout_sec * 1000.0));

  const auto scene_object_ids =
      get_planning_scene_object_ids(node, planning_scene_client, service_call_timeout);
  const std::string original_target_id = request->target.id;
  request->target.id =
      resolve_target_object_id(scene_object_ids, original_target_id);

  if (request->target.id != original_target_id) {
    RCLCPP_INFO(
        node->get_logger(),
        "Resolved target object '%s' to planning scene collision object '%s'",
        original_target_id.c_str(), request->target.id.c_str());
  }

  RCLCPP_INFO(
      node->get_logger(),
      "Received grasp planning request for group: '%s', target object: '%s'",
      request->group_name.c_str(), request->target.id.c_str());

  // These names are defined by robot_hardware_moveit_config.
  const std::string eef_group = request->end_effector_group.empty()
                                    ? "hand"
                                    : request->end_effector_group;

  const std::string ik_frame =
      request->tool_frame.empty() ? "hande_tcp" : request->tool_frame;

  const double planning_timeout_sec =
      request->planning_timeout_sec > 0.0 ? request->planning_timeout_sec : 2.0;

  Eigen::Vector3d box_dims;
  Eigen::Isometry3d obj_t_obj_center = Eigen::Isometry3d::Identity();

  const bool has_explicit_dimensions = request->obj_dims_in_meters.x > 0.0 &&
                                       request->obj_dims_in_meters.y > 0.0 &&
                                       request->obj_dims_in_meters.z > 0.0;

  if (has_explicit_dimensions) {
    box_dims = Eigen::Vector3d(request->obj_dims_in_meters.x,
                               request->obj_dims_in_meters.y,
                               request->obj_dims_in_meters.z);
    obj_t_obj_center = pose_to_isometry(request->obj_t_obj_center);

    RCLCPP_INFO(
        node->get_logger(),
        "Using object dimensions from request. '%s': [%.4f, %.4f, %.4f]",
        request->target.id.c_str(), box_dims.x(), box_dims.y(), box_dims.z());
  } else {
    const auto collision_object = get_planning_scene_collision_object(
        node, planning_scene_client, request->target.id, service_call_timeout);
    if (!collision_object.has_value()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Cannot determine dimensions for target object '%s': the "
                   "object was not found in the planning scene.",
                   request->target.id.c_str());
      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME;
      return;
    }

    const auto bounding_box =
        computeObjectBoundingBox(collision_object.value());

    if (!bounding_box.has_value()) {
      RCLCPP_ERROR(
          node->get_logger(),
          "Cannot determine the dimensions for object '%s': its collision "
          "geometry does not contain a valid BOX primitive or mesh.",
          request->target.id.c_str());
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return;
    }

    box_dims = bounding_box->dimensions;
    obj_t_obj_center = bounding_box->center_transform;

    RCLCPP_INFO(node->get_logger(),
                "Retrieved dimensions for object '%s' from the planning scene: "
                "[%.4f, %.4f, %.4f]",
                request->target.id.c_str(), box_dims.x(), box_dims.y(),
                box_dims.z());
  }

  std::vector<int32_t> surfaces;
  for (int32_t s : request->surfaces) {
    if (s >= 0 && s <= 5) {
      surfaces.push_back(s);
    }
  }
  if (surfaces.empty()) {
    surfaces = {0, 1, 2, 3, 4, 5};
  }

  const int num_rotations =
      request->num_rotations > 0 ? request->num_rotations : 4;

  const std::string pregrasp_pose = "open";
  constexpr std::size_t kMaxGrasps = 10;

  response->grasps.clear();

  if (request->group_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), "group_name is empty in the request.");
    response->error_code.val =
        moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return;
  }

  try {
    mtc::Task task;
    task.stages()->setName("grasp_generator_task");
    task.loadRobotModel(node);

    const auto robot_model = task.getRobotModel();
    if (robot_model->getEndEffector(eef_group) == nullptr ||
        robot_model->getLinkModel(ik_frame) == nullptr) {
      RCLCPP_ERROR(
          node->get_logger(),
          "Robot model does not define end effector '%s' with IK frame '%s'.",
          eef_group.c_str(), ik_frame.c_str());
      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      return;
    }

    task.setProperty("group", request->group_name);
    task.setProperty("eef", eef_group);
    task.setProperty("ik_frame", ik_frame);

    /*
    With current state we can get the robot state and the planning scene.
    request->target.id must exist in the scene previously.
    */
    mtc::Stage* current_state_ptr = nullptr;

    {
      auto current_state =
          std::make_unique<mtc::stages::CurrentState>("current_state");
      current_state_ptr = current_state.get();
      task.add(std::move(current_state));
    }

    {
      auto hand_planner =
          std::make_shared<mtc::solvers::JointInterpolationPlanner>();
      auto open_hand =
          std::make_unique<mtc::stages::MoveTo>("open hand", hand_planner);

      open_hand->setGroup(eef_group);
      open_hand->setGoal(pregrasp_pose);

      task.add(std::move(open_hand));
    }

    /*
    Connect plans a collision-free trajectory from the current robot state
    to the pre-grasp state generated by the relative approach stage.
    */

    {
      auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);

      auto connect = std::make_unique<mtc::stages::Connect>(
          "connect current state to pre-grasp",
          mtc::stages::Connect::GroupPlannerVector{
              {request->group_name, planner}});

      connect->setTimeout(planning_timeout_sec);
      connect->properties().configureInitFrom(mtc::Stage::PARENT);

      task.add(std::move(connect));
    }

    mtc::Stage* approach_stage_ptr = nullptr;
    {
      auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

      cartesian_planner->setStepSize(0.01);
      cartesian_planner->setMaxVelocityScalingFactor(1.0);
      cartesian_planner->setMaxAccelerationScalingFactor(1.0);

      auto approach = std::make_unique<mtc::stages::MoveRelative>(
          "approach object", cartesian_planner);

      approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      approach->setGroup(request->group_name);
      approach->setIKFrame(ik_frame);

      const double retract_dist =
          request->retract_dist_m > 0.0 ? request->retract_dist_m : 0.05;
      approach->setMinMaxDistance(retract_dist * 0.5, retract_dist);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = ik_frame;
      direction.vector.x = 0.0;
      direction.vector.y = 0.0;
      direction.vector.z = 1.0;

      approach->setDirection(direction);

      approach_stage_ptr = approach.get();
      task.add(std::move(approach));
    }

    {
      auto allow_gripper_object_collision =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "allow gripper-object collision");

      const auto* eef_joint_model_group =
          robot_model->getJointModelGroup(eef_group);

      allow_gripper_object_collision->allowCollisions(
          request->target.id,
          eef_joint_model_group->getLinkModelNamesWithCollisionGeometry(),
          true);

      task.add(std::move(allow_gripper_object_collision));
    }

    {
      auto generator =
          std::make_unique<GenerateBoxGraspPoses>("generate box grasp poses");

      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->properties().set("marker_ns", "grasp_poses");

      generator->setObject(request->target.id);
      generator->setPreGraspPose(pregrasp_pose);
      generator->setSurfaces(surfaces);
      generator->setNumRotations(num_rotations);
      generator->setBoxDimensions(box_dims);
      generator->setCenterTransform(obj_t_obj_center);
      generator->setMonitoredStage(current_state_ptr);

      auto compute_ik = std::make_unique<mtc::stages::ComputeIK>(
          "compute grasp ik", std::move(generator));

      compute_ik->setMaxIKSolutions(8);
      compute_ik->setMinSolutionDistance(0.1);
      compute_ik->setIKFrame(ik_frame);

      compute_ik->properties().configureInitFrom(mtc::Stage::PARENT,
                                                 {"eef", "group"});
      compute_ik->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                                 {"target_pose"});

      compute_ik->setForwardedProperties(
          {"target_pose"});  // share the target pose with the next stage

      task.add(std::move(compute_ik));
    }

    task.init();

    const auto planning_result = task.plan(kMaxGrasps);

    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS ||
        task.solutions().empty()) {
      RCLCPP_WARN(node->get_logger(),
                  "No reachable grasp candidates found for object '%s'.",
                  request->target.id.c_str());

      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return;
    }

    const auto* arm_jmg = robot_model->getJointModelGroup(request->group_name);
    if (arm_jmg == nullptr) {
      RCLCPP_ERROR(node->get_logger(),
                   "Arm joint model group '%s' not found in robot model.",
                   request->group_name.c_str());
      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return;
    }
    const auto arm_active_joint_names = arm_jmg->getActiveJointModelNames();

    const double retract_dist =
        request->retract_dist_m > 0.0 ? request->retract_dist_m : 0.05;

    moveit_msgs::msg::GripperTranslation pre_grasp_approach;
    pre_grasp_approach.direction.header.stamp = node->now();
    pre_grasp_approach.direction.header.frame_id = ik_frame;
    pre_grasp_approach.direction.vector.x = 0.0;
    pre_grasp_approach.direction.vector.y = 0.0;
    pre_grasp_approach.direction.vector.z = 1.0;
    pre_grasp_approach.desired_distance = static_cast<float>(retract_dist);
    pre_grasp_approach.min_distance = static_cast<float>(retract_dist * 0.5);

    moveit_msgs::msg::GripperTranslation post_grasp_retreat;
    post_grasp_retreat.direction.header.stamp = node->now();
    post_grasp_retreat.direction.header.frame_id = ik_frame;
    post_grasp_retreat.direction.vector.x = 0.0;
    post_grasp_retreat.direction.vector.y = 0.0;
    post_grasp_retreat.direction.vector.z = -1.0;
    post_grasp_retreat.desired_distance = static_cast<float>(retract_dist);
    post_grasp_retreat.min_distance = static_cast<float>(retract_dist * 0.5);

    moveit_msgs::msg::GripperTranslation post_place_retreat;
    post_place_retreat.direction.header.stamp = node->now();
    post_place_retreat.direction.header.frame_id = ik_frame;
    post_place_retreat.direction.vector.x = 0.0;
    post_place_retreat.direction.vector.y = 0.0;
    post_place_retreat.direction.vector.z = -1.0;
    post_place_retreat.desired_distance = static_cast<float>(retract_dist);
    post_place_retreat.min_distance = static_cast<float>(retract_dist * 0.5);

    std::vector<moveit_msgs::msg::Grasp> valid_grasps;
    valid_grasps.reserve(task.solutions().size());

    std::vector<geometry_msgs::msg::PoseStamped> valid_pregrasp_poses;
    valid_pregrasp_poses.reserve(task.solutions().size());

    for (const auto& solution : task.solutions()) {
      if (!solution || solution->isFailure() || solution->end() == nullptr) {
        continue;
      }

      try {
        const auto& target_pose =
            solution->end()->properties().get<geometry_msgs::msg::PoseStamped>(
                "target_pose");

        // 1. Grasp Posture: populated with the arm's IK solution at the grasp
        // pose
        const auto& grasp_robot_state =
            solution->end()->scene()->getCurrentState();
        std::vector<double> grasp_arm_positions;
        grasp_robot_state.copyJointGroupPositions(arm_jmg, grasp_arm_positions);

        trajectory_msgs::msg::JointTrajectory grasp_posture;
        grasp_posture.joint_names = arm_active_joint_names;
        grasp_posture.header.stamp = node->now();
        grasp_posture.header.frame_id = robot_model->getModelFrame();

        trajectory_msgs::msg::JointTrajectoryPoint grasp_point;
        grasp_point.positions = grasp_arm_positions;
        grasp_point.time_from_start.sec = 0;
        grasp_point.time_from_start.nanosec = 500000000;
        grasp_posture.points.push_back(grasp_point);

        // 2. Pre-Grasp Posture: populated with the arm's IK solution at the
        // pre-grasp pose
        std::vector<double> pregrasp_arm_positions;
        bool found_pregrasp = false;
        Eigen::Isometry3d pregrasp_pose_world = Eigen::Isometry3d::Identity();
        const Eigen::Isometry3d world_t_object =
            solution->end()->scene()->getFrameTransform(
                target_pose.header.frame_id);

        const auto* approach_sol =
            findSubSolution(solution.get(), approach_stage_ptr);
        if (approach_sol != nullptr && approach_sol->start() != nullptr &&
            approach_sol->start()->scene() != nullptr) {
          const auto& pregrasp_robot_state =
              approach_sol->start()->scene()->getCurrentState();
          pregrasp_robot_state.copyJointGroupPositions(arm_jmg,
                                                       pregrasp_arm_positions);
          pregrasp_pose_world =
              pregrasp_robot_state.getFrameTransform(ik_frame);
          found_pregrasp = true;
        } else {
          // Fallback: solve IK for pre-grasp pose using grasp state as seed
          moveit::core::RobotState pregrasp_robot_state(grasp_robot_state);
          Eigen::Isometry3d target_pose_world =
              world_t_object * pose_to_isometry(target_pose.pose);
          // Retract along -Z of ik_frame
          pregrasp_pose_world =
              target_pose_world * Eigen::Translation3d(0.0, 0.0, -retract_dist);
          if (pregrasp_robot_state.setFromIK(arm_jmg, pregrasp_pose_world,
                                             ik_frame, 0.1)) {
            pregrasp_robot_state.copyJointGroupPositions(
                arm_jmg, pregrasp_arm_positions);
            found_pregrasp = true;
          }
        }

        if (!found_pregrasp) {
          RCLCPP_WARN(node->get_logger(),
                      "Failed to compute pregrasp IK solution or approach "
                      "state; rejecting candidate.");
          continue;
        }

        Eigen::Isometry3d object_t_pregrasp =
            world_t_object.inverse() * pregrasp_pose_world;

        geometry_msgs::msg::PoseStamped pre_grasp_pose_msg;
        pre_grasp_pose_msg.header.stamp = node->now();
        pre_grasp_pose_msg.header.frame_id = target_pose.header.frame_id;
        pre_grasp_pose_msg.pose = isometry_to_pose(object_t_pregrasp);

        trajectory_msgs::msg::JointTrajectory pre_grasp_posture;
        pre_grasp_posture.joint_names = arm_active_joint_names;
        pre_grasp_posture.header.stamp = node->now();
        pre_grasp_posture.header.frame_id = robot_model->getModelFrame();

        trajectory_msgs::msg::JointTrajectoryPoint pre_point;
        pre_point.positions = pregrasp_arm_positions;
        pre_point.time_from_start.sec = 0;
        pre_point.time_from_start.nanosec = 500000000;
        pre_grasp_posture.points.push_back(pre_point);

        moveit_msgs::msg::Grasp grasp;
        grasp.id = "grasp_" + std::to_string(valid_grasps.size());
        grasp.pre_grasp_posture = pre_grasp_posture;
        grasp.grasp_posture = grasp_posture;
        grasp.grasp_pose = target_pose;
        const double cost = std::max(0.0, solution->cost());
        grasp.grasp_quality = 1.0 / (1.0 + cost);
        grasp.pre_grasp_approach = pre_grasp_approach;
        grasp.post_grasp_retreat = post_grasp_retreat;
        grasp.post_place_retreat = post_place_retreat;
        grasp.max_contact_force = 0.0f;
        grasp.allowed_touch_objects = {request->target.id};

        RCLCPP_INFO(
            node->get_logger(), "Candidate %s: cost = %.4f (comment: '%s')",
            grasp.id.c_str(), solution->cost(), solution->comment().c_str());

        valid_grasps.push_back(std::move(grasp));
        valid_pregrasp_poses.push_back(std::move(pre_grasp_pose_msg));

        if (valid_grasps.size() >= kMaxGrasps) {
          break;
        }

      } catch (const std::exception& exception) {
        RCLCPP_WARN(node->get_logger(),
                    "Complete MTC solution does not contain target_pose: %s",
                    exception.what());
      }
    }

    if (valid_grasps.empty()) {
      RCLCPP_WARN(node->get_logger(),
                  "Complete solutions were found, but no grasp poses could be "
                  "extracted.");

      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return;
    }

    task.introspection().publishSolution(*task.solutions().front());

    response->grasps = std::move(valid_grasps);
    response->pre_grasp_poses = std::move(valid_pregrasp_poses);
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    RCLCPP_INFO(node->get_logger(),
                "Successfully generated %zu reachable grasp candidates for "
                "object '%s'.",
                response->grasps.size(), request->target.id.c_str());

  } catch (const mtc::InitStageException& exception) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "MTC initialization failed:\n"
                                                << exception);

    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(node->get_logger(), "Grasp planning failed: %s",
                 exception.what());

    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
}

double get_double_parameter(const rclcpp::Node::SharedPtr& node,
                            const std::string& name, double default_value) {
  if (!node->has_parameter(name)) {
    return default_value;
  }
  const auto& param = node->get_parameter(name);
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return param.as_double();
  } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(param.as_int());
  }
  return default_value;
}

/**
 * @brief Continually retries calling flowstate_ros_bridge and verifying
 * MoveIt's planning scene until populated.
 */
void trigger_add_collision_objects(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<std::atomic<bool>>& scene_ready) {
  const double ros_service_call_timeout_sec =
      get_double_parameter(node, "ros_service_call_timeout_sec", 5.0);
  const double retry_interval_sec =
      get_double_parameter(node, "add_collision_retry_interval_sec", 2.0);
  const bool expect_collision_objects =
      node->get_parameter("expect_collision_objects").as_bool();

  const auto service_call_timeout = std::chrono::milliseconds(
      static_cast<long long>(ros_service_call_timeout_sec * 1000.0));
  const auto retry_interval = std::chrono::milliseconds(
      static_cast<long long>(retry_interval_sec * 1000.0));

  std::thread([node, scene_ready, service_call_timeout, retry_interval,
               expect_collision_objects]() {
    auto bridge_client = node->create_client<std_srvs::srv::Trigger>(
        "/flowstate_ros_bridge/add_collision_objects");
    auto get_scene_client =
        node->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/get_planning_scene");

    while (rclcpp::ok() && !scene_ready->load()) {
      // Step 1: Ensure MoveIt's move_group node and planning scene monitor
      // service are up
      if (!get_scene_client->wait_for_service(retry_interval)) {
        RCLCPP_INFO(
            node->get_logger(),
            "Waiting for MoveIt '/get_planning_scene' service (move_group)...");
        continue;
      }

      if (!expect_collision_objects) {
        RCLCPP_INFO(node->get_logger(),
                    "'expect_collision_objects' is false. Proceeding without "
                    "collision scene population.");
        scene_ready->store(true);
        break;
      }

      // Step 2: Ensure flowstate_ros_bridge service is up
      if (!bridge_client->wait_for_service(retry_interval)) {
        RCLCPP_INFO(node->get_logger(),
                    "Waiting for '/flowstate_ros_bridge/add_collision_objects' "
                    "service...");
        continue;
      }

      // Step 3: Trigger flowstate_ros_bridge to publish collision objects
      RCLCPP_INFO(node->get_logger(),
                  "Requesting collision objects from "
                  "'/flowstate_ros_bridge/add_collision_objects'...");

      auto bridge_req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto bridge_future = bridge_client->async_send_request(bridge_req);

      if (bridge_future.wait_for(service_call_timeout) !=
          std::future_status::ready) {
        RCLCPP_WARN(node->get_logger(),
                    "Timeout waiting for add_collision_objects response. "
                    "Retrying in %.1fs...",
                    retry_interval.count() / 1000.0);
        std::this_thread::sleep_for(retry_interval);
        continue;
      }

      try {
        auto bridge_resp = bridge_future.get();
        if (!bridge_resp->success) {
          RCLCPP_WARN(node->get_logger(),
                      "Bridge reported failure populating collision objects: "
                      "%s. Retrying in %.1fs...",
                      bridge_resp->message.c_str(),
                      retry_interval.count() / 1000.0);
          std::this_thread::sleep_for(retry_interval);
          continue;
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Exception calling add_collision_objects: %s. Retrying in %.1fs...",
            e.what(), retry_interval.count() / 1000.0);
        std::this_thread::sleep_for(retry_interval);
        continue;
      }

      if (!expect_collision_objects) {
        RCLCPP_INFO(node->get_logger(),
                    "Successfully called add_collision_objects. "
                    "'expect_collision_objects' is false, skipping scene "
                    "verification.");
        scene_ready->store(true);
        break;
      }

      // Step 4: Give ROS 2 graph discovery / topic transport a short moment to
      // deliver messages
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // Step 5: Query MoveIt's planning scene to verify collision objects exist
      auto scene_req =
          std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
      scene_req->components.components =
          moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;

      auto scene_future = get_scene_client->async_send_request(scene_req);
      if (scene_future.wait_for(service_call_timeout) ==
          std::future_status::ready) {
        try {
          auto scene_resp = scene_future.get();
          const auto& objects = scene_resp->scene.world.collision_objects;
          if (!objects.empty()) {
            RCLCPP_INFO(node->get_logger(),
                        "Successfully verified %zu collision objects in MoveIt "
                        "planning scene!",
                        objects.size());
            scene_ready->store(true);
            break;
          } else {
            RCLCPP_WARN(node->get_logger(),
                        "MoveIt planning scene is still empty after triggering "
                        "collision objects. Retrying in %.1fs...",
                        retry_interval.count() / 1000.0);
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(node->get_logger(),
                       "Exception querying MoveIt planning scene: %s. Retrying "
                       "in %.1fs...",
                       e.what(), retry_interval.count() / 1000.0);
        }
      } else {
        RCLCPP_WARN(
            node->get_logger(),
            "Timeout querying MoveIt planning scene. Retrying in %.1fs...",
            retry_interval.count() / 1000.0);
      }

      std::this_thread::sleep_for(retry_interval);
    }
  }).detach();
}

intrinsic_proto::config::RuntimeContext GetRuntimeContext() {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file;
  runtime_context_file.open("/etc/intrinsic/runtime_config.pb",
                            std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    // Return default context for running locally
    std::cerr << "Warning: using default RuntimeContext\n";
  }
  return runtime_context;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto runtime_context = GetRuntimeContext();
  intrinsic::MoveItPlanningServiceConfig config;
  if (!runtime_context.config().UnpackTo(&config)) {
    RCLCPP_WARN(
        rclcpp::get_logger("moveit_planning_service"),
        "Failed to unpack MoveItPlanningServiceConfig from runtime_config.pb; "
        "using default parameters.");
  }

  std::vector<rclcpp::Parameter> params;
  if (config.ros_service_call_timeout_sec() > 0.0) {
    params.emplace_back("ros_service_call_timeout_sec",
                        config.ros_service_call_timeout_sec());
  }
  if (config.add_collision_retry_interval_sec() > 0.0) {
    params.emplace_back("add_collision_retry_interval_sec",
                        config.add_collision_retry_interval_sec());
  }
  if (config.has_expect_collision_objects()) {
    params.emplace_back("expect_collision_objects",
                        config.expect_collision_objects());
  }
  if (config.has_use_mock_hardware()) {
    params.emplace_back("use_mock_hardware", config.use_mock_hardware());
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  auto node = rclcpp::Node::make_shared("moveit_planning_node", options);

  rcl_interfaces::msg::ParameterDescriptor double_desc;
  double_desc.dynamic_typing = true;

  node->declare_parameter("ros_service_call_timeout_sec",
                          rclcpp::ParameterValue(5.0), double_desc);
  node->declare_parameter("add_collision_retry_interval_sec",
                          rclcpp::ParameterValue(2.0), double_desc);
  node->declare_parameter<bool>("expect_collision_objects", true);
  node->declare_parameter<bool>("use_mock_hardware", false);

  auto scene_ready = std::make_shared<std::atomic<bool>>(false);

  auto planning_scene_callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

  auto planning_scene_client =
      node->create_client<moveit_msgs::srv::GetPlanningScene>(
          "/get_planning_scene", rclcpp::ServicesQoS(),
          planning_scene_callback_group);

  auto planning_scene_executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  planning_scene_executor->add_callback_group(planning_scene_callback_group,
                                              node->get_node_base_interface());

  std::thread planning_scene_executor_thread(
      [planning_scene_executor]() { planning_scene_executor->spin(); });

  auto motion_service = node->create_service<moveit_msgs::srv::GetMotionPlan>(
      "motion_planning/get_motion_plan",
      [scene_ready](
          const std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request>
              request,
          std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Response> response) {
        handle_motion_plan_request(scene_ready, request, response);
      });

  auto grasp_service = node->create_service<PlanGrasps>(
      "grasp_planning/plan_grasps",
      [node, planning_scene_client, scene_ready](
          const PlanGrasps::Request::SharedPtr request,
          PlanGrasps::Response::SharedPtr response) {
        handle_grasp_planning_request(node, planning_scene_client, scene_ready,
                                      request, response);
      });

  RCLCPP_INFO(node->get_logger(), "MoveIt Planning Service started.");
  RCLCPP_INFO(node->get_logger(), "Ready to receive requests at:");
  RCLCPP_INFO(node->get_logger(), " - motion_planning/get_motion_plan");
  RCLCPP_INFO(node->get_logger(), " - grasp_planning/plan_grasps");

  trigger_add_collision_objects(node, scene_ready);

  rclcpp::experimental::executors::EventsExecutor executor;
  executor.add_node(node);
  executor.spin();

  planning_scene_executor->cancel();

  if (planning_scene_executor_thread.joinable()) {
    planning_scene_executor_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
