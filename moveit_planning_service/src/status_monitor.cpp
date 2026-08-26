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

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include "absl/status/status.h"
#include "absl/synchronization/notification.h"
#include "grpcpp/grpcpp.h"
#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/util/grpc/grpc.h"

#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_planning_interfaces/srv/plan_grasps.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace {

constexpr char const* kGetPlanningSceneTopic = "/get_planning_scene";
constexpr char const* kBridgeAddCollisionTopic =
    "/flowstate_ros_bridge/add_collision_objects";
constexpr char const* kMotionPlanTopic = "motion_planning/get_motion_plan";
constexpr char const* kGraspPlanTopic = "grasp_planning/plan_grasps";

struct SubsystemHealthReport {
  bool healthy = false;
  std::string title;
  std::string message;
  std::string instructions;
};

}  // namespace

class MoveItStatusClient : public rclcpp::Node {
 public:
  MoveItStatusClient()
      : Node("moveit_status_monitor_client"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    this->declare_parameter<std::string>("planning_frame", "world");
    this->declare_parameter<std::string>("tool_frame", "hande_tcp");
    this->declare_parameter<bool>("expect_collision_objects", true);
    rcl_interfaces::msg::ParameterDescriptor double_desc;
    double_desc.dynamic_typing = true;
    this->declare_parameter("service_call_timeout_sec",
                            rclcpp::ParameterValue(2.0), double_desc);

    get_scene_client_ =
        this->create_client<moveit_msgs::srv::GetPlanningScene>(
            kGetPlanningSceneTopic);
    bridge_client_ =
        this->create_client<std_srvs::srv::Trigger>(kBridgeAddCollisionTopic);
    motion_plan_client_ =
        this->create_client<moveit_msgs::srv::GetMotionPlan>(kMotionPlanTopic);
    grasp_plan_client_ =
        this->create_client<moveit_planning_interfaces::srv::PlanGrasps>(
            kGraspPlanTopic);
  }

  bool is_disabled() const { return is_disabled_.load(); }
  void set_disabled(bool disabled) { is_disabled_.store(disabled); }

  SubsystemHealthReport check_health() {
    SubsystemHealthReport report;
    double timeout_sec = 2.0;
    if (this->has_parameter("service_call_timeout_sec")) {
      const auto& p = this->get_parameter("service_call_timeout_sec");
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        timeout_sec = p.as_double();
      } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        timeout_sec = static_cast<double>(p.as_int());
      }
    }
    const auto timeout = std::chrono::milliseconds(
        static_cast<long long>(timeout_sec * 1000.0));
    const bool expect_collision_objects =
        this->get_parameter("expect_collision_objects").as_bool();
    const std::string planning_frame =
        this->get_parameter("planning_frame").as_string();
    const std::string tool_frame =
        this->get_parameter("tool_frame").as_string();

    // 1. Verify move_group is responsive via /get_planning_scene
    if (!get_scene_client_->wait_for_service(timeout)) {
      report.healthy = false;
      report.title = "MoveIt move_group is Offline";
      report.message =
          "The MoveIt '/get_planning_scene' service is unreachable or not responding.";
      report.instructions =
          "Ensure move_group is launched, robot_description parameters are valid, "
          "and MoveIt planning pipelines loaded successfully.";
      return report;
    }

    auto scene_req =
        std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    scene_req->components.components =
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    auto scene_future = get_scene_client_->async_send_request(scene_req);

    if (scene_future.wait_for(timeout) != std::future_status::ready) {
      report.healthy = false;
      report.title = "MoveIt Planning Scene Query Timed Out";
      report.message =
          "MoveIt move_group did not respond to the planning scene query within timeout.";
      report.instructions =
          "Check move_group logs for deadlock or heavy planning tasks blocking the node executor.";
      return report;
    }

    std::size_t num_collision_objects = 0;
    try {
      auto scene_resp = scene_future.get();
      num_collision_objects = scene_resp->scene.world.collision_objects.size();
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
          "The moveit_planning_node endpoints ('motion_planning/get_motion_plan', "
          "'grasp_planning/plan_grasps') are not registered.";
      report.instructions =
          "Ensure moveit_planning_node is running and has completed initialization.";
      return report;
    }

    // 3. Verify collision objects in planning scene if expected
    if (expect_collision_objects && num_collision_objects == 0) {
      report.healthy = false;
      report.title = "Planning Scene Empty";
      report.message =
          "MoveIt planning scene has 0 collision objects registered.";
      report.instructions =
          "Ensure flowstate_ros_bridge is running and trigger Enable to sync collision objects.";
      return report;
    }

    // 4. Check TF tree connectivity
    std::string tf_err;
    if (!tool_frame.empty() && !planning_frame.empty()) {
      if (!tf_buffer_.canTransform(planning_frame, tool_frame,
                                   tf2::TimePointZero, &tf_err)) {
        report.healthy = false;
        report.title = "TF Tree Incomplete";
        report.message =
            std::string("Cannot transform from planning frame '" + planning_frame +
            "' to tool frame '" + tool_frame + "': " + tf_err);
        report.instructions =
            "Ensure robot_state_publisher and flowstate_ros_bridge are running "
            "and publishing transforms for the robot tool and base links.";
        return report;
      }
    }

    report.healthy = true;
    return report;
  }

  bool trigger_scene_synchronization(std::chrono::milliseconds timeout = 5000ms) {
    if (!bridge_client_->wait_for_service(timeout)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Bridge service '%s' is unreachable.",
                   kBridgeAddCollisionTopic);
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = bridge_client_->async_send_request(request);

    if (future.wait_for(timeout) != std::future_status::ready) {
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
                   "Exception calling bridge add_collision_objects: %s",
                   e.what());
      return false;
    }

    return true;
  }

 private:
  std::atomic<bool> is_disabled_{false};
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr
      get_scene_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr bridge_client_;
  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr motion_plan_client_;
  rclcpp::Client<moveit_planning_interfaces::srv::PlanGrasps>::SharedPtr
      grasp_plan_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

class ServiceStateImpl
    : public intrinsic_proto::services::v1::ServiceState::Service {
 public:
  explicit ServiceStateImpl(std::shared_ptr<MoveItStatusClient> client)
      : client_(client) {}

  ::grpc::Status GetState(
      ::grpc::ServerContext* /*context*/,
      const ::intrinsic_proto::services::v1::GetStateRequest* /*request*/,
      ::intrinsic_proto::services::v1::SelfState* response) override {
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

  ::grpc::Status Enable(
      ::grpc::ServerContext* /*context*/,
      const ::intrinsic_proto::services::v1::EnableRequest* /*request*/,
      ::intrinsic_proto::services::v1::EnableResponse* /*response*/) override {
    client_->set_disabled(false);

    // Trigger bridge collision scene synchronization
    if (!client_->trigger_scene_synchronization(5000ms)) {
      return ::grpc::Status(
          ::grpc::StatusCode::INTERNAL,
          "Failed to synchronize collision scene via flowstate_ros_bridge");
    }

    // Give ROS 2 transport a brief moment to deliver messages to MoveIt
    std::this_thread::sleep_for(500ms);

    auto report = client_->check_health();
    if (!report.healthy) {
      return ::grpc::Status(::grpc::StatusCode::INTERNAL,
                            "Enable failed: " + report.title + " - " +
                                report.message);
    }

    return ::grpc::Status::OK;
  }

  ::grpc::Status Disable(
      ::grpc::ServerContext* /*context*/,
      const ::intrinsic_proto::services::v1::DisableRequest* /*request*/,
      ::intrinsic_proto::services::v1::DisableResponse* /*response*/) override {
    client_->set_disabled(true);
    return ::grpc::Status::OK;
  }

 private:
  std::shared_ptr<MoveItStatusClient> client_;
};

intrinsic_proto::config::RuntimeContext GetRuntimeContext() {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file("/etc/intrinsic/runtime_config.pb",
                                     std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    std::cerr << "Warning: using default RuntimeContext\n";
  }
  return runtime_context;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto client = std::make_shared<MoveItStatusClient>();

  auto runtime_context = GetRuntimeContext();

  std::vector<::grpc::Service*> services;
  ServiceStateImpl health_service(client);
  services.push_back(&health_service);

  auto server_or = intrinsic::CreateServer(runtime_context.port(), services);
  if (!server_or.ok()) {
    std::cerr << "Failed to start gRPC server: " << server_or.status().message()
              << "\n";
    return 1;
  }
  auto server = std::move(server_or.value());

  std::cout << "MoveIt Status Monitor gRPC server listening on port "
            << runtime_context.port() << "\n";

  // Spin the ROS 2 node in a background thread
  std::thread spin_thread([client]() { rclcpp::spin(client); });

  absl::Notification registered;
  auto status = intrinsic::RegisterSignalHandlerAndWait(
      server.get(), intrinsic::ShutdownParams::Aggressive(), registered);

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
