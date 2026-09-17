// Copyright 2026 Intrinsic Innovation LLC

#ifndef MOVEIT_PLANNING_SERVICE_PUBLISH_STATIC_TRANSFORMS_HPP_
#define MOVEIT_PLANNING_SERVICE_PUBLISH_STATIC_TRANSFORMS_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace moveit_planning_service {

struct StaticTransformsConfig {
  bool publish_world_root_tf = true;
  bool publish_robot_base_tf = true;
  std::string world_frame = "world";
  std::string robot_base_frame = "ur_module/base_link";
};

/**
 * @brief Broadcasts static transforms to bridge world/root and robot
 * base frames.
 *
 * Broadcasts:
 *  1. [world_frame -> root] (identity transform) if publish_world_root_tf is
 * true.
 *  2. [robot_base_frame -> base_link] (180-degree yaw rotation around Z) if
 * publish_robot_base_tf is true.
 *
 * @param node Shared pointer to the ROS 2 node.
 * @param config Configuration options specifying which static transforms to
 * publish and frame IDs.
 * @return Broadcaster instance maintaining ownership of static transform
 * publications.
 */
std::unique_ptr<tf2_ros::StaticTransformBroadcaster> PublishStaticTransforms(
    const rclcpp::Node::SharedPtr& node, const StaticTransformsConfig& config);

}  // namespace moveit_planning_service

#endif  // MOVEIT_PLANNING_SERVICE_PUBLISH_STATIC_TRANSFORMS_HPP_
