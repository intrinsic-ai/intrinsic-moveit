// Copyright 2026 Intrinsic Innovation LLC

#include "moveit_planning_service/grasp_planning_pipeline.hpp"

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
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <map>
#include <memory>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit_msgs/msg/grasp.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <mutex>
#include <optional>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <utility>
#include <vector>

#include "moveit_planning_service/generate_box_grasp_poses.hpp"
#include "moveit_planning_service/object_geometry.hpp"

namespace moveit_planning_service {

namespace mtc = moveit::task_constructor;

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

GraspPlanningPipeline::GraspPlanningPipeline(
    rclcpp::Node::SharedPtr node,
    PlanningSceneClient::SharedPtr planning_scene_client,
    moveit::core::RobotModelConstPtr robot_model)
    : node_(std::move(node)),
      planning_scene_client_(std::move(planning_scene_client)),
      robot_model_(std::move(robot_model)) {
  if (!robot_model_ && node_ && node_->has_parameter("robot_description")) {
    try {
      robot_model_loader_ =
          std::make_shared<robot_model_loader::RobotModelLoader>(
              node_, "robot_description");
      robot_model_ = robot_model_loader_->getModel();
    } catch (const std::exception& e) {
      RCLCPP_WARN(
          node_->get_logger(),
          "Failed to load RobotModel in GraspPlanningPipeline constructor: %s",
          e.what());
    }
  }
}

moveit::core::RobotModelConstPtr GraspPlanningPipeline::GetRobotModel() {
  if (!robot_model_ && node_) {
    std::lock_guard<std::mutex> lock(robot_model_mutex_);
    if (!robot_model_) {
      try {
        robot_model_loader_ =
            std::make_shared<robot_model_loader::RobotModelLoader>(
                node_, "robot_description");
        robot_model_ = robot_model_loader_->getModel();
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load RobotModel: %s",
                     e.what());
      }
    }
  }
  return robot_model_;
}

std::string GraspPlanningPipeline::ResolveTargetObjectId(
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

  // 3. Canonical entity suffix (<clean_id>/whole)
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

std::vector<std::string> GraspPlanningPipeline::GetPlanningSceneObjectIds(
    std::chrono::milliseconds timeout) const {
  std::vector<std::string> object_ids;
  if (!planning_scene_client_ || !node_) {
    return object_ids;
  }

  if (!planning_scene_client_->service_is_ready()) {
    if (!planning_scene_client_->wait_for_service(timeout)) {
      return object_ids;
    }
  }

  auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  req->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;

  auto future = planning_scene_client_->async_send_request(req);
  if (future.wait_for(timeout) != std::future_status::ready) {
    planning_scene_client_->remove_pending_request(future);
    RCLCPP_WARN(node_->get_logger(),
                "Timed out while retrieving planning scene object IDs.");
    return object_ids;
  }
  try {
    const auto resp = future.get();
    for (const auto& obj : resp->scene.world.collision_objects) {
      object_ids.push_back(obj.id);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to retrieve planning scene object IDs: %s", e.what());
  }
  return object_ids;
}

std::optional<moveit_msgs::msg::CollisionObject>
GraspPlanningPipeline::GetPlanningSceneCollisionObject(
    const std::string& object_id, std::chrono::milliseconds timeout) const {
  if (!planning_scene_client_ || !node_) {
    return std::nullopt;
  }

  if (!planning_scene_client_->service_is_ready() &&
      !planning_scene_client_->wait_for_service(timeout)) {
    RCLCPP_WARN(
        node_->get_logger(),
        "Planning scene service is unavailable while retrieving object '%s'.",
        object_id.c_str());
    return std::nullopt;
  }

  auto request =
      std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

  auto future = planning_scene_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    planning_scene_client_->remove_pending_request(future);
    RCLCPP_WARN(node_->get_logger(),
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
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to retrieve collision object '%s': %s",
                 object_id.c_str(), e.what());
    return std::nullopt;
  }

  RCLCPP_WARN(node_->get_logger(),
              "Collision object '%s' was not found in the planning scene.",
              object_id.c_str());
  return std::nullopt;
}

void GraspPlanningPipeline::PlanGrasps(
    const std::shared_ptr<PlanGraspsSrv::Request>& request,
    std::shared_ptr<PlanGraspsSrv::Response>& response,
    std::chrono::milliseconds service_call_timeout) {
  if (!node_) {
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }

  if (request->target.id.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "target.id is empty.");
    response->error_code.val =
        moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME;
    return;
  }

  // Resolve target object ID against active planning scene
  const auto scene_object_ids = GetPlanningSceneObjectIds(service_call_timeout);
  const std::string original_target_id = request->target.id;
  request->target.id =
      ResolveTargetObjectId(scene_object_ids, original_target_id);

  if (request->target.id != original_target_id) {
    RCLCPP_INFO(
        node_->get_logger(),
        "Resolved target object '%s' to planning scene collision object '%s'",
        original_target_id.c_str(), request->target.id.c_str());
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "Received grasp planning request for group: '%s', target object: '%s'",
      request->group_name.c_str(), request->target.id.c_str());

  const std::string eef_group = request->end_effector_group.empty()
                                    ? "hand"
                                    : request->end_effector_group;

  const std::string ik_frame =
      request->tool_frame.empty() ? "hande_tcp" : request->tool_frame;

  const double planning_timeout_sec =
      request->planning_timeout_sec > 0.0 ? request->planning_timeout_sec : 5.0;

  const double gripper_motion_duration_sec =
      request->gripper_motion_duration_sec > 0.0
          ? request->gripper_motion_duration_sec
          : 0.5;

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
        node_->get_logger(),
        "Using object dimensions from request. '%s': [%.4f, %.4f, %.4f]",
        request->target.id.c_str(), box_dims.x(), box_dims.y(), box_dims.z());
  } else {
    const auto collision_object = GetPlanningSceneCollisionObject(
        request->target.id, service_call_timeout);
    if (!collision_object.has_value()) {
      RCLCPP_ERROR(node_->get_logger(),
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
          node_->get_logger(),
          "Cannot determine the dimensions for object '%s': its collision "
          "geometry does not contain a valid BOX primitive or mesh.",
          request->target.id.c_str());
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return;
    }

    box_dims = bounding_box->dimensions;
    obj_t_obj_center = bounding_box->center_transform;

    RCLCPP_INFO(node_->get_logger(),
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
    RCLCPP_ERROR(node_->get_logger(), "group_name is empty in the request.");
    response->error_code.val =
        moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return;
  }

  try {
    const auto robot_model = GetRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(node_->get_logger(), "Robot model could not be loaded.");
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return;
    }

    mtc::Task task;
    task.stages()->setName("grasp_generator_task");
    task.setRobotModel(robot_model);

    if (robot_model->getEndEffector(eef_group) == nullptr ||
        robot_model->getLinkModel(ik_frame) == nullptr) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Robot model does not define end effector '%s' with IK frame '%s'.",
          eef_group.c_str(), ik_frame.c_str());
      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      return;
    }

    task.setProperty("group", request->group_name);
    task.setProperty("eef", eef_group);
    task.setProperty("ik_frame", ik_frame);

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

    {
      auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
      planner->setProperty("num_planning_attempts", 5u);

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
      if (eef_joint_model_group != nullptr) {
        allow_gripper_object_collision->allowCollisions(
            request->target.id,
            eef_joint_model_group->getLinkModelNamesWithCollisionGeometry(),
            true);
      }

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

      compute_ik->setTimeout(0.2);
      compute_ik->setMaxIKSolutions(16);
      compute_ik->setMinSolutionDistance(0.1);
      compute_ik->setIKFrame(ik_frame);

      compute_ik->properties().configureInitFrom(mtc::Stage::PARENT,
                                                 {"eef", "group"});
      compute_ik->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                                 {"target_pose"});

      compute_ik->setForwardedProperties({"target_pose"});

      task.add(std::move(compute_ik));
    }

    task.init();

    const auto planning_result = task.plan(kMaxGrasps);

    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS ||
        task.solutions().empty()) {
      RCLCPP_WARN(node_->get_logger(),
                  "No reachable grasp candidates found for object '%s'.",
                  request->target.id.c_str());

      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return;
    }

    const auto* arm_jmg = robot_model->getJointModelGroup(request->group_name);
    if (arm_jmg == nullptr) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Arm joint model group '%s' not found in robot model.",
                   request->group_name.c_str());
      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return;
    }
    const auto arm_active_joint_names = arm_jmg->getActiveJointModelNames();

    const auto* eef_joint_model_group =
        robot_model->getJointModelGroup(eef_group);
    if (eef_joint_model_group == nullptr) {
      RCLCPP_ERROR(node_->get_logger(),
                   "End-effector joint model group '%s' not found.",
                   eef_group.c_str());
      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return;
    }

    std::map<std::string, double> open_gripper_position_map;
    if (!eef_joint_model_group->getVariableDefaultPositions(
            "open", open_gripper_position_map)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Named state 'open' not found for group '%s'.",
                   eef_group.c_str());
      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      return;
    }

    std::map<std::string, double> closed_gripper_position_map;
    if (!eef_joint_model_group->getVariableDefaultPositions(
            "closed", closed_gripper_position_map)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Named state 'closed' not found for group '%s'.",
                   eef_group.c_str());
      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      return;
    }

    const double retract_dist =
        request->retract_dist_m > 0.0 ? request->retract_dist_m : 0.05;

    const auto gripper_joint_names =
        eef_joint_model_group->getActiveJointModelNames();

    std::vector<double> open_gripper_positions;
    std::vector<double> closed_gripper_positions;
    open_gripper_positions.reserve(gripper_joint_names.size());
    closed_gripper_positions.reserve(gripper_joint_names.size());

    for (const auto& joint_name : gripper_joint_names) {
      const auto open_position = open_gripper_position_map.find(joint_name);
      const auto closed_position = closed_gripper_position_map.find(joint_name);

      if (open_position == open_gripper_position_map.end() ||
          closed_position == closed_gripper_position_map.end()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Named gripper states do not contain joint '%s'.",
                     joint_name.c_str());
        response->error_code.val =
            moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return;
      }

      open_gripper_positions.push_back(open_position->second);
      closed_gripper_positions.push_back(closed_position->second);
    }

    const auto gripper_motion_duration =
        rclcpp::Duration::from_seconds(gripper_motion_duration_sec);

    trajectory_msgs::msg::JointTrajectory open_gripper_posture;
    open_gripper_posture.header.stamp = node_->now();
    open_gripper_posture.header.frame_id = robot_model->getModelFrame();
    open_gripper_posture.joint_names = gripper_joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint open_gripper_point;
    open_gripper_point.positions = open_gripper_positions;
    open_gripper_point.time_from_start = gripper_motion_duration;
    open_gripper_posture.points.push_back(std::move(open_gripper_point));

    trajectory_msgs::msg::JointTrajectory closed_gripper_posture;
    closed_gripper_posture.header.stamp = node_->now();
    closed_gripper_posture.header.frame_id = robot_model->getModelFrame();
    closed_gripper_posture.joint_names = gripper_joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint closed_gripper_point;
    closed_gripper_point.positions = closed_gripper_positions;
    closed_gripper_point.time_from_start = gripper_motion_duration;
    closed_gripper_posture.points.push_back(std::move(closed_gripper_point));

    moveit_msgs::msg::GripperTranslation pre_grasp_approach;
    pre_grasp_approach.direction.header.stamp = node_->now();
    pre_grasp_approach.direction.header.frame_id = ik_frame;
    pre_grasp_approach.direction.vector.x = 0.0;
    pre_grasp_approach.direction.vector.y = 0.0;
    pre_grasp_approach.direction.vector.z = 1.0;
    pre_grasp_approach.desired_distance = static_cast<float>(retract_dist);
    pre_grasp_approach.min_distance = static_cast<float>(retract_dist * 0.5);

    moveit_msgs::msg::GripperTranslation post_grasp_retreat;
    post_grasp_retreat.direction.header.stamp = node_->now();
    post_grasp_retreat.direction.header.frame_id = ik_frame;
    post_grasp_retreat.direction.vector.x = 0.0;
    post_grasp_retreat.direction.vector.y = 0.0;
    post_grasp_retreat.direction.vector.z = -1.0;
    post_grasp_retreat.desired_distance = static_cast<float>(retract_dist);
    post_grasp_retreat.min_distance = static_cast<float>(retract_dist * 0.5);

    moveit_msgs::msg::GripperTranslation post_place_retreat;
    post_place_retreat.direction.header.stamp = node_->now();
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

    std::vector<sensor_msgs::msg::JointState> valid_grasp_ik_solutions;
    valid_grasp_ik_solutions.reserve(task.solutions().size());

    std::vector<sensor_msgs::msg::JointState> valid_pregrasp_ik_solutions;
    valid_pregrasp_ik_solutions.reserve(task.solutions().size());

    for (const auto& solution : task.solutions()) {
      if (!solution || solution->isFailure() || solution->end() == nullptr ||
          solution->end()->scene() == nullptr) {
        continue;
      }

      try {
        const auto& target_pose =
            solution->end()->properties().get<geometry_msgs::msg::PoseStamped>(
                "target_pose");

        // Arm IK solution at the grasp pose.
        const auto& grasp_robot_state =
            solution->end()->scene()->getCurrentState();

        sensor_msgs::msg::JointState grasp_ik_solution;
        grasp_ik_solution.header.stamp = node_->now();
        grasp_ik_solution.header.frame_id = robot_model->getModelFrame();
        grasp_ik_solution.name = arm_active_joint_names;
        grasp_robot_state.copyJointGroupPositions(arm_jmg,
                                                  grasp_ik_solution.position);

        // Arm IK solution at the pre-grasp pose.
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
          RCLCPP_WARN(node_->get_logger(),
                      "Failed to compute pregrasp IK solution or approach "
                      "state; rejecting candidate.");
          continue;
        }

        Eigen::Isometry3d object_t_pregrasp =
            world_t_object.inverse() * pregrasp_pose_world;

        geometry_msgs::msg::PoseStamped pre_grasp_pose_msg;
        pre_grasp_pose_msg.header.stamp = node_->now();
        pre_grasp_pose_msg.header.frame_id = target_pose.header.frame_id;
        pre_grasp_pose_msg.pose = isometry_to_pose(object_t_pregrasp);

        sensor_msgs::msg::JointState pregrasp_ik_solution;
        pregrasp_ik_solution.header.stamp = node_->now();
        pregrasp_ik_solution.header.frame_id = robot_model->getModelFrame();
        pregrasp_ik_solution.name = arm_active_joint_names;
        pregrasp_ik_solution.position = std::move(pregrasp_arm_positions);

        moveit_msgs::msg::Grasp grasp;
        grasp.id = "grasp_" + std::to_string(valid_grasps.size());
        grasp.pre_grasp_posture = open_gripper_posture;
        grasp.grasp_posture = closed_gripper_posture;
        grasp.grasp_pose = target_pose;
        const double cost = std::max(0.0, solution->cost());
        grasp.grasp_quality = 1.0 / (1.0 + cost);
        grasp.pre_grasp_approach = pre_grasp_approach;
        grasp.post_grasp_retreat = post_grasp_retreat;
        grasp.post_place_retreat = post_place_retreat;
        grasp.max_contact_force = 0.0f;
        grasp.allowed_touch_objects = {request->target.id};

        RCLCPP_INFO(
            node_->get_logger(), "Candidate %s: cost = %.4f (comment: '%s')",
            grasp.id.c_str(), solution->cost(), solution->comment().c_str());

        valid_grasps.push_back(std::move(grasp));
        valid_pregrasp_poses.push_back(std::move(pre_grasp_pose_msg));

        valid_grasp_ik_solutions.push_back(std::move(grasp_ik_solution));
        valid_pregrasp_ik_solutions.push_back(std::move(pregrasp_ik_solution));

        if (valid_grasps.size() >= kMaxGrasps) {
          break;
        }

      } catch (const std::exception& exception) {
        RCLCPP_WARN(node_->get_logger(),
                    "Complete MTC solution does not contain target_pose: %s",
                    exception.what());
      }
    }

    if (valid_grasps.empty()) {
      RCLCPP_WARN(node_->get_logger(),
                  "Complete solutions were found, but no grasp poses could be "
                  "extracted.");

      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return;
    }

    task.introspection().publishSolution(*task.solutions().front());

    response->grasps = std::move(valid_grasps);
    response->pre_grasp_poses = std::move(valid_pregrasp_poses);
    response->grasp_ik_solutions = std::move(valid_grasp_ik_solutions);
    response->pregrasp_ik_solutions = std::move(valid_pregrasp_ik_solutions);
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    RCLCPP_INFO(node_->get_logger(),
                "Successfully generated %zu reachable grasp candidates for "
                "object '%s'.",
                response->grasps.size(), request->target.id.c_str());

  } catch (const mtc::InitStageException& exception) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "MTC initialization failed:\n"
                                                 << exception);

    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(node_->get_logger(), "Grasp planning failed: %s",
                 exception.what());

    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
}

}  // namespace moveit_planning_service
