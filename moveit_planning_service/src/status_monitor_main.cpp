// Copyright 2026 Intrinsic Innovation LLC

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "absl/status/status.h"
#include "absl/synchronization/notification.h"
#include "grpcpp/grpcpp.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/util/grpc/grpc.h"
#include "moveit_planning_service.pb.h"
#include "moveit_planning_service/status_monitor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

intrinsic_proto::config::RuntimeContext GetRuntimeContext() {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file("/etc/intrinsic/runtime_config.pb",
                                     std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    std::cerr << "Warning: using default RuntimeContext\n";
  }
  return runtime_context;
}

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto runtime_context = GetRuntimeContext();
  intrinsic::MoveItPlanningServiceConfig config;
  if (!runtime_context.config().UnpackTo(&config)) {
    RCLCPP_WARN(
        rclcpp::get_logger("moveit_status_monitor"),
        "Failed to unpack MoveItPlanningServiceConfig from runtime_config.pb; "
        "using default parameters.");
  }

  std::vector<rclcpp::Parameter> params;
  if (config.has_ros_service_call_timeout_sec()) {
    params.emplace_back("ros_service_call_timeout_sec",
                        config.ros_service_call_timeout_sec());
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

  auto client =
      std::make_shared<moveit_planning_service::MoveItStatusClient>(options);

  std::vector<::grpc::Service*> services;
  moveit_planning_service::ServiceStateImpl health_service(client);
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
