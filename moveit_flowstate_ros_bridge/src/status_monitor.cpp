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

#include "absl/status/status.h"
#include "absl/synchronization/notification.h"
#include "grpcpp/grpcpp.h"
#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/util/grpc/grpc.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class BridgeStatusClient : public rclcpp::Node {
 public:
  BridgeStatusClient() : Node("moveit_bridge_status_monitor_client") {
    this->declare_parameter<std::string>("bridge_lifecycle_node",
                                         "/flowstate_ros_bridge");
    std::string bridge_node =
        this->get_parameter("bridge_lifecycle_node").as_string();

    std::string get_state_topic = bridge_node + "/get_state";
    std::string change_state_topic = bridge_node + "/change_state";

    get_state_client_ =
        this->create_client<lifecycle_msgs::srv::GetState>(get_state_topic);
    change_state_client_ =
        this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_topic);
  }

  unsigned int get_bridge_state(std::chrono::milliseconds timeout = 2000ms) {
    if (!get_state_client_->wait_for_service(timeout)) {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = get_state_client_->async_send_request(request);

    if (future.wait_for(timeout) != std::future_status::ready) {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    try {
      return future.get()->current_state.id;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error getting bridge state: %s",
                   e.what());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  bool change_bridge_state(std::uint8_t transition,
                           std::chrono::milliseconds timeout = 5000ms) {
    if (!change_state_client_->wait_for_service(timeout)) {
      return false;
    }

    auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto future = change_state_client_->async_send_request(request);

    if (future.wait_for(timeout) != std::future_status::ready) {
      return false;
    }

    try {
      return future.get()->success;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error changing bridge state: %s",
                   e.what());
      return false;
    }
  }

  bool wait_for_state(std::uint8_t target_state,
                      std::chrono::milliseconds timeout = 2000ms) {
    auto start = std::chrono::steady_clock::now();

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start < timeout) {
      if (get_bridge_state(100ms) == target_state) {
        return true;
      }
      loop_rate.sleep();
    }
    return false;
  }

 private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>
      get_state_client_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
      change_state_client_;
};

class ServiceStateImpl
    : public intrinsic_proto::services::v1::ServiceState::Service {
 public:
  explicit ServiceStateImpl(std::shared_ptr<BridgeStatusClient> client)
      : client_(client) {}

  ::grpc::Status GetState(
      ::grpc::ServerContext* /*context*/,
      const ::intrinsic_proto::services::v1::GetStateRequest* /*request*/,
      ::intrinsic_proto::services::v1::SelfState* response) override {
    unsigned int state_id = client_->get_bridge_state();

    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      response->set_state_code(
          ::intrinsic_proto::services::v1::SelfState::STATE_CODE_ENABLED);
    } else if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
      response->set_state_code(
          ::intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR);
      auto* ext = response->mutable_extended_status();
      ext->set_title("Flowstate ROS Bridge is Offline");
      auto* user = ext->mutable_user_report();
      user->set_message(
          "The flowstate_ros_bridge lifecycle node is unreachable or offline.");
      user->set_instructions(
          "Ensure flowstate_ros_bridge is launched, Zenoh router and service "
          "tunnel are reachable, and bridge configuration is valid.");
    } else {
      response->set_state_code(
          ::intrinsic_proto::services::v1::SelfState::STATE_CODE_DISABLED);
    }
    return ::grpc::Status::OK;
  }

  ::grpc::Status Enable(
      ::grpc::ServerContext* /*context*/,
      const ::intrinsic_proto::services::v1::EnableRequest* /*request*/,
      ::intrinsic_proto::services::v1::EnableResponse* /*response*/) override {
    unsigned int state_id = client_->get_bridge_state();
    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      if (!client_->change_bridge_state(
              lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
        return ::grpc::Status(::grpc::StatusCode::INTERNAL,
                              "Failed to configure bridge node");
      }
      if (!client_->wait_for_state(
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)) {
        return ::grpc::Status(
            ::grpc::StatusCode::INTERNAL,
            "Failed to wait for bridge node to become inactive");
      }
      state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    }

    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      if (!client_->change_bridge_state(
              lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
        return ::grpc::Status(::grpc::StatusCode::INTERNAL,
                              "Failed to activate bridge node");
      }
      if (!client_->wait_for_state(
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)) {
        return ::grpc::Status(
            ::grpc::StatusCode::INTERNAL,
            "Failed to wait for bridge node to become active");
      }
    }
    return ::grpc::Status::OK;
  }

  ::grpc::Status Disable(
      ::grpc::ServerContext* /*context*/,
      const ::intrinsic_proto::services::v1::DisableRequest* /*request*/,
      ::intrinsic_proto::services::v1::DisableResponse* /*response*/) override {
    unsigned int state_id = client_->get_bridge_state();
    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      if (!client_->change_bridge_state(
              lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
        return ::grpc::Status(::grpc::StatusCode::INTERNAL,
                              "Failed to deactivate bridge node");
      }
      if (!client_->wait_for_state(
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)) {
        return ::grpc::Status(
            ::grpc::StatusCode::INTERNAL,
            "Failed to wait for bridge node to become inactive");
      }
      state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    }

    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      if (!client_->change_bridge_state(
              lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
        return ::grpc::Status(::grpc::StatusCode::INTERNAL,
                              "Failed to cleanup bridge node");
      }
      if (!client_->wait_for_state(
              lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)) {
        return ::grpc::Status(
            ::grpc::StatusCode::INTERNAL,
            "Failed to wait for bridge node to become unconfigured");
      }
    }
    return ::grpc::Status::OK;
  }

 private:
  std::shared_ptr<BridgeStatusClient> client_;
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
  auto client = std::make_shared<BridgeStatusClient>();

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

  std::cout << "Bridge status monitor gRPC server listening on port "
            << runtime_context.port() << "\n";

  // Spin the node in a background thread
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
