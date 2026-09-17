// Copyright 2026 Intrinsic Innovation LLC

#include "moveit_planning_service/status_monitor.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace moveit_planning_service {
namespace {

class StatusMonitorTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(StatusMonitorTest, UninitializedServiceReportsOfflineMoveGroup) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"use_mock_hardware", false},
      {"ros_service_call_timeout_sec", 0.05},
  });

  auto client = std::make_shared<MoveItStatusClient>(options);
  EXPECT_FALSE(client->is_disabled());

  const auto report = client->check_health();
  EXPECT_FALSE(report.healthy);
  EXPECT_EQ(report.title, "MoveIt move_group is Offline");
}

TEST_F(StatusMonitorTest, DisableAndEnableStateToggles) {
  rclcpp::NodeOptions options;
  auto client = std::make_shared<MoveItStatusClient>(options);

  EXPECT_FALSE(client->is_disabled());
  client->set_disabled(true);
  EXPECT_TRUE(client->is_disabled());
  client->set_disabled(false);
  EXPECT_FALSE(client->is_disabled());
}

TEST_F(StatusMonitorTest, RecordJointStateUpdatesInternalCache) {
  rclcpp::NodeOptions options;
  auto client = std::make_shared<MoveItStatusClient>(options);

  sensor_msgs::msg::JointState js;
  js.header.stamp = client->now();
  js.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  js.position = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};

  EXPECT_NO_THROW(client->record_joint_state(js));
}

TEST_F(StatusMonitorTest, MockHardwareModeFlag) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"use_mock_hardware", true},
  });

  auto client = std::make_shared<MoveItStatusClient>(options);
  EXPECT_TRUE(client->is_mock_hardware());
}

}  // namespace
}  // namespace moveit_planning_service
