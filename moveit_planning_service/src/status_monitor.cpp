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

#include "moveit_planning_service/status_monitor.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "absl/strings/str_format.h"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_planning_interfaces/srv/plan_grasps.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace moveit_planning_service {

namespace {

constexpr char const* kGetPlanningSceneTopic = "/get_planning_scene";
constexpr char const* kSyncCollisionObjectsTopic =
    "/moveit_planning_node/sync_collision_objects";
constexpr char const* kMotionPlanTopic = "motion_planning/get_motion_plan";
constexpr char const* kGraspPlanTopic = "grasp_planning/plan_grasps";

}  // namespace

MoveItStatusClient::MoveItStatusClient(const rclcpp::NodeOptions& options)
    : Node("moveit_status_monitor_client", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  this->declare_parameter<std::string>("planning_frame", "world");
  this->declare_parameter<std::string>("robot_base_frame", "base_link");
  this->declare_parameter<std::string>("tool_frame", "hande_tcp");
  this->declare_parameter<bool>("expect_collision_objects", true);
  this->declare_parameter<bool>("use_mock_hardware", false);
  this->declare_parameter<std::vector<std::string>>(
      "expected_joints",
      std::vector<std::string>{"shoulder_pan_joint", "shoulder_lift_joint",
                               "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                               "wrist_3_joint"});
  rcl_interfaces::msg::ParameterDescriptor double_desc;
  double_desc.dynamic_typing = true;
  this->declare_parameter("ros_service_call_timeout_sec",
                          rclcpp::ParameterValue(2.0), double_desc);
  this->declare_parameter("joint_state_timeout_sec",
                          rclcpp::ParameterValue(3.0), double_desc);

  get_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>(
      kGetPlanningSceneTopic);
  bridge_client_ =
      this->create_client<std_srvs::srv::Trigger>(kSyncCollisionObjectsTopic);
  motion_plan_client_ =
      this->create_client<moveit_msgs::srv::GetMotionPlan>(kMotionPlanTopic);
  grasp_plan_client_ =
      this->create_client<moveit_planning_interfaces::srv::PlanGrasps>(
          kGraspPlanTopic);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg) {
          record_joint_state(*msg);
        }
      });
}

void MoveItStatusClient::record_joint_state(
    const sensor_msgs::msg::JointState& msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  last_joint_state_time_ = this->now();
  received_joint_names_.clear();
  for (const auto& name : msg.name) {
    received_joint_names_.insert(name);
  }
}

SubsystemHealthReport MoveItStatusClient::check_health() {
  SubsystemHealthReport report;
  double timeout_sec = 2.0;
  if (this->has_parameter("ros_service_call_timeout_sec")) {
    const auto& p = this->get_parameter("ros_service_call_timeout_sec");
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      timeout_sec = p.as_double();
    } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      timeout_sec = static_cast<double>(p.as_int());
    }
  }
  const auto timeout =
      std::chrono::milliseconds(static_cast<long long>(timeout_sec * 1000.0));

  double js_timeout_sec = 3.0;
  if (this->has_parameter("joint_state_timeout_sec")) {
    const auto& p = this->get_parameter("joint_state_timeout_sec");
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      js_timeout_sec = p.as_double();
    } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      js_timeout_sec = static_cast<double>(p.as_int());
    }
  }

  const bool expect_collision_objects =
      this->get_parameter("expect_collision_objects").as_bool();
  const bool use_mock_hardware =
      this->get_parameter("use_mock_hardware").as_bool();
  const std::string planning_frame =
      this->get_parameter("planning_frame").as_string();
  const std::string robot_base_frame =
      this->get_parameter("robot_base_frame").as_string();
  const std::string tool_frame = this->get_parameter("tool_frame").as_string();
  const std::vector<std::string> expected_joints =
      this->get_parameter("expected_joints").as_string_array();

  // 1. Verify move_group is responsive via /get_planning_scene
  if (!get_scene_client_->wait_for_service(timeout)) {
    report.healthy = false;
    report.title = "MoveIt move_group is Offline";
    report.message =
        "The MoveIt '/get_planning_scene' service is unreachable "
        "or not responding.";
    report.instructions =
        "Ensure move_group is launched, robot_description "
        "parameters are valid, "
        "and MoveIt planning pipelines loaded successfully.";
    return report;
  }

  auto scene_req =
      std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  scene_req->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE;
  auto scene_future = get_scene_client_->async_send_request(scene_req);

  if (scene_future.wait_for(timeout) != std::future_status::ready) {
    get_scene_client_->remove_pending_request(scene_future);
    report.healthy = false;
    report.title = "MoveIt Planning Scene Query Timed Out";
    report.message =
        "MoveIt move_group did not respond to the planning scene "
        "query within timeout.";
    report.instructions =
        "Check move_group logs for deadlock or heavy "
        "planning tasks blocking the node executor.";
    return report;
  }

  std::size_t num_collision_objects = 0;
  std::size_t num_robot_state_joints = 0;
  try {
    auto scene_resp = scene_future.get();
    num_collision_objects = scene_resp->scene.world.collision_objects.size();
    num_robot_state_joints =
        scene_resp->scene.robot_state.joint_state.name.size();
  } catch (const std::exception& e) {
    report.healthy = false;
    report.title = "MoveIt Planning Scene Exception";
    report.message =
        std::string("Exception occurred when querying planning scene: ") +
        e.what();
    report.instructions =
        "Check MoveIt configuration and node logs for details.";
    return report;
  }

  // 2. Verify moveit_planning_node endpoints exist
  if (!motion_plan_client_->service_is_ready() ||
      !grasp_plan_client_->service_is_ready()) {
    report.healthy = false;
    report.title = "Planning Service Node Offline";
    report.message =
        "The moveit_planning_node endpoints "
        "('motion_planning/get_motion_plan', "
        "'grasp_planning/plan_grasps') are not registered.";
    report.instructions =
        "Ensure moveit_planning_node is running and has "
        "completed initialization.";
    return report;
  }

  // 3. Verify collision objects in planning scene if expected
  if (!use_mock_hardware && expect_collision_objects &&
      num_collision_objects == 0) {
    report.healthy = false;
    report.title = "Planning Scene Empty";
    report.message =
        "MoveIt planning scene has 0 collision objects registered.";
    report.instructions =
        "Ensure moveit_planning_service is connected to World service and "
        "trigger Enable to sync collision objects.";
    return report;
  }

  // 4. Verify Joint States health (only when not in mock hardware mode)
  if (!use_mock_hardware) {
    rclcpp::Time last_js_time;
    absl::flat_hash_set<std::string> js_names;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      last_js_time = last_joint_state_time_;
      js_names = received_joint_names_;
    }

    if (last_js_time.nanoseconds() == 0) {
      report.healthy = false;
      report.title = "Joint States Offline";
      report.message =
          "/joint_states topic has not received any messages. "
          "Ensure flowstate_ros_bridge is running.";
      report.instructions =
          "Launch upstream bridge via 'ros2 launch moveit_planning_service "
          "flowstate_ros_bridge.launch.py'.";
      return report;
    }

    const double js_age_sec = (this->now() - last_js_time).seconds();
    if (js_age_sec > js_timeout_sec) {
      report.healthy = false;
      report.title = "Joint States Stale";
      report.message = absl::StrFormat(
          "/joint_states stream is stale (last "
          "received %.1fs ago, timeout is %.1fs).",
          js_age_sec, js_timeout_sec);
      report.instructions =
          "Check flowstate_ros_bridge logs and Zenoh "
          "router connectivity.";
      return report;
    }

    for (const auto& expected_joint : expected_joints) {
      if (!js_names.contains(expected_joint)) {
        report.healthy = false;
        report.title = "Missing Expected Joints";
        report.message = absl::StrFormat(
            "Expected joint '%s' was not found in /joint_states stream.",
            expected_joint);
        report.instructions =
            "Check robot description model and "
            "override_joint_names in flowstate_ros_bridge.";
        return report;
      }
    }

    // 5. Verify MoveIt's internal robot state has received joint updates
    if (num_robot_state_joints == 0) {
      report.healthy = false;
      report.title = "MoveIt Robot State Uninitialized";
      report.message =
          "MoveIt move_group planning scene robot_state has 0 "
          "joint states populated.";
      report.instructions =
          "Ensure joint_state_broadcaster or flowstate_ros_bridge is "
          "publishing to /joint_states.";
      return report;
    }
  }

  // 6. Check TF tree connectivity (planning_frame -> robot_base_frame ->
  // tool_frame)
  std::string tf_err;
  if (!robot_base_frame.empty() && !planning_frame.empty()) {
    if (!tf_buffer_.canTransform(planning_frame, robot_base_frame,
                                 tf2::TimePointZero, &tf_err)) {
      report.healthy = false;
      report.title = "TF Tree Incomplete";
      report.message = absl::StrFormat(
          "Cannot transform from planning frame '%s' to robot "
          "base frame '%s': %s",
          planning_frame, robot_base_frame, tf_err);
      report.instructions =
          "Ensure moveit_planning_service static transforms "
          "and flowstate_ros_bridge are running.";
      return report;
    }
  }

  if (!tool_frame.empty() && !robot_base_frame.empty()) {
    if (!tf_buffer_.canTransform(robot_base_frame, tool_frame,
                                 tf2::TimePointZero, &tf_err)) {
      report.healthy = false;
      report.title = "TF Tree Incomplete";
      report.message = absl::StrFormat(
          "Cannot transform from robot base frame '%s' to tool frame '%s': %s",
          robot_base_frame, tool_frame, tf_err);
      report.instructions =
          "Ensure robot_state_publisher is receiving "
          "/joint_states and broadcasting tool transforms.";
      return report;
    }
  }

  report.healthy = true;
  return report;
}

bool MoveItStatusClient::trigger_scene_synchronization(
    std::chrono::milliseconds timeout) {
  if (!bridge_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(this->get_logger(), "Bridge service '%s' is unreachable.",
                 kSyncCollisionObjectsTopic);
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = bridge_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    bridge_client_->remove_pending_request(future);
    RCLCPP_ERROR(this->get_logger(),
                 "Timeout waiting for bridge to populate collision objects.");
    return false;
  }

  try {
    auto resp = future.get();
    if (!resp->success) {
      RCLCPP_ERROR(this->get_logger(),
                   "Bridge returned failure populating collision objects: %s",
                   resp->message.c_str());
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception calling bridge to request collision object "
                 "synchronization: %s",
                 e.what());
    return false;
  }

  return true;
}

ServiceStateImpl::ServiceStateImpl(std::shared_ptr<MoveItStatusClient> client)
    : client_(client) {}

::grpc::Status ServiceStateImpl::GetState(
    ::grpc::ServerContext* /*context*/,
    const ::intrinsic_proto::services::v1::GetStateRequest* /*request*/,
    ::intrinsic_proto::services::v1::SelfState* response) {
  if (client_->is_disabled()) {
    response->set_state_code(
        ::intrinsic_proto::services::v1::SelfState::STATE_CODE_DISABLED);
    return ::grpc::Status::OK;
  }

  auto report = client_->check_health();
  if (report.healthy) {
    response->set_state_code(
        ::intrinsic_proto::services::v1::SelfState::STATE_CODE_ENABLED);
  } else {
    response->set_state_code(
        ::intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR);
    auto* ext = response->mutable_extended_status();
    ext->set_title(report.title);
    auto* user = ext->mutable_user_report();
    user->set_message(report.message);
    user->set_instructions(report.instructions);
  }

  return ::grpc::Status::OK;
}

::grpc::Status ServiceStateImpl::Enable(
    ::grpc::ServerContext* /*context*/,
    const ::intrinsic_proto::services::v1::EnableRequest* /*request*/,
    ::intrinsic_proto::services::v1::EnableResponse* /*response*/) {
  client_->set_disabled(false);

  // Trigger collision scene synchronization if not using mock hardware
  if (!client_->is_mock_hardware()) {
    if (!client_->trigger_scene_synchronization(5000ms)) {
      return ::grpc::Status(
          ::grpc::StatusCode::INTERNAL,
          "Failed to synchronize collision scene in moveit_planning_service");
    }
  }

  // Give ROS 2 transport a brief moment to deliver messages to MoveIt
  std::this_thread::sleep_for(500ms);

  auto report = client_->check_health();
  if (!report.healthy) {
    return ::grpc::Status(
        ::grpc::StatusCode::INTERNAL,
        "Enable failed: " + report.title + " - " + report.message);
  }

  return ::grpc::Status::OK;
}

::grpc::Status ServiceStateImpl::Disable(
    ::grpc::ServerContext* /*context*/,
    const ::intrinsic_proto::services::v1::DisableRequest* /*request*/,
    ::intrinsic_proto::services::v1::DisableResponse* /*response*/) {
  client_->set_disabled(true);
  return ::grpc::Status::OK;
}

}  // namespace moveit_planning_service
