// Copyright 2026 Intrinsic Innovation LLC

#include "moveit_planning_service/publish_static_transforms.hpp"

#include <vector>

#include "absl/log/log.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace moveit_planning_service {

std::unique_ptr<tf2_ros::StaticTransformBroadcaster> PublishStaticTransforms(
    const rclcpp::Node::SharedPtr& node, const StaticTransformsConfig& config) {
  if (!node) {
    LOG(ERROR) << "PublishStaticTransforms called with null node pointer.";
    return nullptr;
  }

  auto static_broadcaster =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

  const std::string world_frame =
      config.world_frame.empty() ? "world" : config.world_frame;
  if (config.publish_world_root_tf) {
    // 1. world_frame -> root (identity transform)
    geometry_msgs::msg::TransformStamped world_root_tf;
    world_root_tf.header.stamp = node->now();
    world_root_tf.header.frame_id = world_frame;
    world_root_tf.child_frame_id = "root";
    world_root_tf.transform.translation.x = 0.0;
    world_root_tf.transform.translation.y = 0.0;
    world_root_tf.transform.translation.z = 0.0;
    world_root_tf.transform.rotation.x = 0.0;
    world_root_tf.transform.rotation.y = 0.0;
    world_root_tf.transform.rotation.z = 0.0;
    world_root_tf.transform.rotation.w = 1.0;
    static_transforms.push_back(world_root_tf);
    RCLCPP_INFO(node->get_logger(), "Publishing static transform: [%s -> root]",
                world_frame.c_str());
  }

  const std::string robot_base_frame = config.robot_base_frame.empty()
                                           ? "ur_module/base_link"
                                           : config.robot_base_frame;
  if (config.publish_robot_base_tf) {
    // 2. robot_base_frame -> base_link (180 degree yaw rotation around Z axis)
    geometry_msgs::msg::TransformStamped robot_base_tf;
    robot_base_tf.header.stamp = node->now();
    robot_base_tf.header.frame_id = robot_base_frame;
    robot_base_tf.child_frame_id = "base_link";
    robot_base_tf.transform.translation.x = 0.0;
    robot_base_tf.transform.translation.y = 0.0;
    robot_base_tf.transform.translation.z = 0.0;
    robot_base_tf.transform.rotation.x = 0.0;
    robot_base_tf.transform.rotation.y = 0.0;
    robot_base_tf.transform.rotation.z = 1.0;
    robot_base_tf.transform.rotation.w = 0.0;
    static_transforms.push_back(robot_base_tf);
    RCLCPP_INFO(node->get_logger(),
                "Publishing static transform: [%s -> base_link]",
                robot_base_frame.c_str());
  }

  if (!static_transforms.empty()) {
    static_broadcaster->sendTransform(static_transforms);
  } else {
    RCLCPP_INFO(node->get_logger(),
                "No static transforms configured to publish.");
  }

  return static_broadcaster;
}

}  // namespace moveit_planning_service
