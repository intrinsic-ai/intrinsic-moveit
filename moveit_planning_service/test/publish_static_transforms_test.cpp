// Copyright 2026 Intrinsic Innovation LLC

#include "moveit_planning_service/publish_static_transforms.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

class PublishStaticTransformsTest : public ::testing::Test {
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

  void SetUp() override {
    node_ =
        std::make_shared<rclcpp::Node>("test_publish_static_transforms_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  void TearDown() override {
    if (executor_) {
      executor_->remove_node(node_);
    }
    executor_.reset();
    node_.reset();
  }

  template <typename Predicate>
  bool SpinUntil(Predicate pred, std::chrono::milliseconds timeout =
                                     std::chrono::milliseconds(500)) {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      executor_->spin_some();
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_->spin_some();
    return pred();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(PublishStaticTransformsTest, NullNodeReturnsNull) {
  moveit_planning_service::StaticTransformsConfig config;
  auto broadcaster =
      moveit_planning_service::PublishStaticTransforms(nullptr, config);
  EXPECT_EQ(broadcaster, nullptr);
}

TEST_F(PublishStaticTransformsTest, PublishAllStaticTransforms) {
  moveit_planning_service::StaticTransformsConfig config;
  config.publish_world_root_tf = true;
  config.publish_robot_base_tf = true;
  config.world_frame = "world";
  config.robot_base_frame = "ur_module/base_link";

  auto broadcaster =
      moveit_planning_service::PublishStaticTransforms(node_, config);
  ASSERT_NE(broadcaster, nullptr);

  tf2_ros::Buffer tf_buffer(node_->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  const bool received_all = SpinUntil([&]() {
    return tf_buffer.canTransform("root", "world", tf2::TimePointZero) &&
           tf_buffer.canTransform("base_link", "ur_module/base_link",
                                  tf2::TimePointZero);
  });
  EXPECT_TRUE(received_all);

  // 1. Verify world -> root transform
  EXPECT_TRUE(tf_buffer.canTransform("root", "world", tf2::TimePointZero));
  geometry_msgs::msg::TransformStamped world_root_tf =
      tf_buffer.lookupTransform("root", "world", tf2::TimePointZero);
  EXPECT_NEAR(world_root_tf.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(world_root_tf.transform.translation.y, 0.0, 1e-6);
  EXPECT_NEAR(world_root_tf.transform.translation.z, 0.0, 1e-6);
  EXPECT_NEAR(std::abs(world_root_tf.transform.rotation.w), 1.0, 1e-6);

  // 2. Verify ur_module/base_link -> base_link transform (180 deg yaw)
  EXPECT_TRUE(tf_buffer.canTransform("base_link", "ur_module/base_link",
                                     tf2::TimePointZero));
  geometry_msgs::msg::TransformStamped robot_base_tf =
      tf_buffer.lookupTransform("base_link", "ur_module/base_link",
                                tf2::TimePointZero);
  EXPECT_NEAR(robot_base_tf.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(robot_base_tf.transform.translation.y, 0.0, 1e-6);
  EXPECT_NEAR(robot_base_tf.transform.translation.z, 0.0, 1e-6);
  EXPECT_NEAR(std::abs(robot_base_tf.transform.rotation.z), 1.0, 1e-6);
  EXPECT_NEAR(robot_base_tf.transform.rotation.w, 0.0, 1e-6);
}

TEST_F(PublishStaticTransformsTest, DisableWorldRootTransform) {
  moveit_planning_service::StaticTransformsConfig config;
  config.publish_world_root_tf = false;
  config.publish_robot_base_tf = true;
  config.robot_base_frame = "ur_module/base_link";

  auto broadcaster =
      moveit_planning_service::PublishStaticTransforms(node_, config);
  ASSERT_NE(broadcaster, nullptr);

  tf2_ros::Buffer tf_buffer(node_->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  const bool received_robot_base = SpinUntil([&]() {
    return tf_buffer.canTransform("base_link", "ur_module/base_link",
                                  tf2::TimePointZero);
  });
  EXPECT_TRUE(received_robot_base);

  EXPECT_FALSE(tf_buffer.canTransform("root", "world", tf2::TimePointZero));
  EXPECT_TRUE(tf_buffer.canTransform("base_link", "ur_module/base_link",
                                     tf2::TimePointZero));
}

TEST_F(PublishStaticTransformsTest, CustomFrames) {
  moveit_planning_service::StaticTransformsConfig config;
  config.publish_world_root_tf = true;
  config.publish_robot_base_tf = true;
  config.world_frame = "custom_world";
  config.robot_base_frame = "custom_base";

  auto broadcaster =
      moveit_planning_service::PublishStaticTransforms(node_, config);
  ASSERT_NE(broadcaster, nullptr);

  tf2_ros::Buffer tf_buffer(node_->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  const bool received_custom = SpinUntil([&]() {
    return tf_buffer.canTransform("root", "custom_world", tf2::TimePointZero) &&
           tf_buffer.canTransform("base_link", "custom_base",
                                  tf2::TimePointZero);
  });
  EXPECT_TRUE(received_custom);

  EXPECT_TRUE(
      tf_buffer.canTransform("root", "custom_world", tf2::TimePointZero));
  EXPECT_TRUE(
      tf_buffer.canTransform("base_link", "custom_base", tf2::TimePointZero));
}
