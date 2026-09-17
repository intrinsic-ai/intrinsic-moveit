// Copyright 2026 Intrinsic Innovation LLC

#ifndef MOVEIT_PLANNING_SERVICE_STATUS_MONITOR_HPP_
#define MOVEIT_PLANNING_SERVICE_STATUS_MONITOR_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "grpcpp/grpcpp.h"
#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_planning_interfaces/srv/plan_grasps.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace moveit_planning_service {

struct SubsystemHealthReport {
  bool healthy = false;
  std::string title;
  std::string message;
  std::string instructions;
};

class MoveItStatusClient : public rclcpp::Node {
 public:
  explicit MoveItStatusClient(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  bool is_disabled() const { return is_disabled_.load(); }
  void set_disabled(bool disabled) { is_disabled_.store(disabled); }
  bool is_mock_hardware() const {
    return this->get_parameter("use_mock_hardware").as_bool();
  }

  SubsystemHealthReport check_health();
  bool trigger_scene_synchronization(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));

  // Diagnostic helper to inject joint state (used in tests or programmatic
  // verification)
  void record_joint_state(const sensor_msgs::msg::JointState& msg);

 private:
  std::atomic<bool> is_disabled_{false};
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr
      get_scene_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr bridge_client_;
  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr
      motion_plan_client_;
  rclcpp::Client<moveit_planning_interfaces::srv::PlanGrasps>::SharedPtr
      grasp_plan_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex joint_state_mutex_;
  rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
  absl::flat_hash_set<std::string> received_joint_names_;
};

class ServiceStateImpl
    : public intrinsic_proto::services::v1::ServiceState::Service {
 public:
  explicit ServiceStateImpl(std::shared_ptr<MoveItStatusClient> client);

  ::grpc::Status GetState(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::services::v1::GetStateRequest* request,
      ::intrinsic_proto::services::v1::SelfState* response) override;

  ::grpc::Status Enable(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::services::v1::EnableRequest* request,
      ::intrinsic_proto::services::v1::EnableResponse* response) override;

  ::grpc::Status Disable(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::services::v1::DisableRequest* request,
      ::intrinsic_proto::services::v1::DisableResponse* response) override;

 private:
  std::shared_ptr<MoveItStatusClient> client_;
};

}  // namespace moveit_planning_service

#endif  // MOVEIT_PLANNING_SERVICE_STATUS_MONITOR_HPP_
