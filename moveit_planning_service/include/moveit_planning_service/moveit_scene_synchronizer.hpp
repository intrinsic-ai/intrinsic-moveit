// Copyright 2026 Intrinsic Innovation LLC

#ifndef MOVEIT_PLANNING_SERVICE_MOVEIT_SCENE_SYNCHRONIZER_HPP_
#define MOVEIT_PLANNING_SERVICE_MOVEIT_SCENE_SYNCHRONIZER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "flowstate_ros_bridge/world.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace moveit_planning_service {

/**
 * @brief Configuration parameters for MoveitSceneSynchronizer.
 */
struct MoveitSceneSynchronizerConfig {
  std::string tf_prefix = "";
  std::vector<std::string> strip_tf_prefixes = {""};
  std::vector<std::string> excluded_collision_namespaces = {
      "ur_module", "gripper", "camera_mount", "orbbec_camera",
      "ecat_ft_hal_module_with_adapters_with_sim"};
  double collision_objects_update_rate_hz = 10.0;
  double collision_objects_min_position_delta = 0.001;  // 1 mm
  double collision_objects_min_rotation_delta = 0.01;   // ~0.57 deg
  double world_sync_interval_sec = 2.0;  // Periodic background sync
};

/**
 * @brief In-process World collision scene synchronizer for MoveIt.
 *
 * Subscribes to World TF stream for object tracking, converts
 * collision geometries (primitives and glTF meshes) into MoveIt
 * CollisionObjects, and publishes collision updates in real time to
 * /collision_object.
 */
class MoveitSceneSynchronizer {
 public:
  MoveitSceneSynchronizer();
  ~MoveitSceneSynchronizer();

  /**
   * @brief Initializes publishers, subscriptions, and services using the
   * provided configuration.
   *
   * @param node Shared pointer to the ROS 2 node.
   * @param world_client Shared pointer to connected World client.
   * @param config Strongly typed synchronization configuration.
   * @return true if initialization succeeded, false otherwise.
   */
  bool initialize(
      const rclcpp::Node::SharedPtr& node,
      const std::shared_ptr<flowstate_ros_bridge::World>& world_client,
      const MoveitSceneSynchronizerConfig& config);

  /**
   * @brief Fetches scene objects from World service and populates the MoveIt
   * planning scene.
   *
   * @param object_frames Optional list of specific object frame IDs to fetch;
   * if nullopt, fetches all.
   * @return absl::Status Ok if objects were retrieved and published
   * successfully.
   */
  absl::Status fetchAndSynchronizeCollisionObjects(
      std::optional<std::vector<std::string>> object_frames = std::nullopt);

  /**
   * @brief Publishes ADD operations for tracked collision objects to MoveIt
   * planning scene.
   *
   * @param object_frames Optional list of specific object frames to add; if
   * nullopt, adds all tracked objects.
   */
  void addCollisionObjectsToScene(
      std::optional<std::vector<std::string>> object_frames = std::nullopt);

  /**
   * @brief Removes a collision object by frame name from MoveIt's planning
   * scene.
   *
   * @param tf_frame_name The TF frame name identifying the collision object.
   */
  void removeCollisionObject(const std::string& tf_frame_name);

  /**
   * @brief Updates tracked collision object poses in MoveIt based on new TF
   * transforms.
   *
   * @param transforms Vector of updated TF transforms.
   */
  void updateCollisionObjects(
      const std::vector<geometry_msgs::msg::TransformStamped>& transforms);

  /**
   * @brief Returns the number of collision objects currently tracked.
   */
  std::size_t getTrackedCollisionObjectsCount() const;

  /**
   * @brief Strips configured prefixes from a frame string.
   */
  static std::string StripTfPrefixes(absl::string_view frame,
                                     const std::vector<std::string>& prefixes);

 private:
  struct Data;
  std::shared_ptr<Data> data_;
};

}  // namespace moveit_planning_service

#endif  // MOVEIT_PLANNING_SERVICE_MOVEIT_SCENE_SYNCHRONIZER_HPP_
