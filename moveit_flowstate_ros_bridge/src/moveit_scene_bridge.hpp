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

#ifndef MOVEIT_SCENE_BRIDGE_HPP_
#define MOVEIT_SCENE_BRIDGE_HPP_

#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_split.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_interfaces/srv/get_resource.hpp"
#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace flowstate_ros_bridge {

///=============================================================================
// The Node will retrieve information required for configuration from
// ROS Parameters.
class MoveitSceneBridge : public BridgeInterface {
 public:
  ~MoveitSceneBridge();

  using GetResource = flowstate_interfaces::srv::GetResource;

  /// Documentation inherited.
  void declare_ros_parameters(ROSNodeInterfaces ros_node_interfaces) final;

  /// Documentation inherited.
  bool initialize(ROSNodeInterfaces ros_node_interfaces,
                  std::shared_ptr<Executive> executive_client,
                  std::shared_ptr<World> world_client) final;

 private:
  void TfCallback(const intrinsic_proto::TFMessage&);

  static std::string StripTfPrefixes(absl::string_view frame,
                                     const std::vector<std::string>& prefixes);

  void RobotStateCallback(const intrinsic_proto::data_logger::LogItem&);

  void HandleRobotStatus(const intrinsic_proto::icon::RobotStatus& robot_status,
                         const rclcpp::Time& time);

  void PublishJointState(const std::string& part_name,
                         const std::string& frame_id,
                         const intrinsic_proto::icon::PartStatus& part_status,
                         const rclcpp::Time& time);

  struct Data : public std::enable_shared_from_this<Data> {
    /**
     * @brief Send visualization messages for Flowstate sceneObjects
     *
     * @param object_names An optional vector of the names of SceneObjects to be
     * retrieved and published as Marker messages. If unspecified, all
     * SceneObjects in the Belief World will be retrieved and published.
     * @return absl::Status
     */
    absl::Status SendObjectVisualizationMessages(
        std::optional<std::vector<std::string>> object_names = std::nullopt);

    /**
     * @brief Add new collision meshes to Moveit's planning scene
     *
     * @param object_frames An optional vector of the tf_frame_names to be
     * added. If unspecified, all tracked collision meshes will be added.
     */
    void addCollisionObjectsToScene(
        std::optional<std::vector<std::string>> object_frames = std::nullopt);

    /**
     * @brief Update collision objects in planning scene with new poses from TF
     */
    void updateCollisionObjects(
        const std::vector<geometry_msgs::msg::TransformStamped>& transforms);

    /**
     * @brief Remove collision object with specified id from planning scene
     *
     * @param tf_frame_name The id associated with the collision object
     */
    void removeCollisionObject(const std::string& tf_frame_name);

    ROSNodeInterfaces node_interfaces_;
    std::shared_ptr<World> world_;

    std::shared_ptr<intrinsic::Subscription> tf_sub_;
    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tf_pub_;

    // Robot state / joint state functionality
    std::shared_ptr<intrinsic::Subscription> robot_state_sub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
        robot_joint_state_pub_;
    bool robot_joint_state_topic_enabled_{true};
    std::string robot_base_frame_id_;
    std::optional<std::string> robot_arm_part_name_;
    std::vector<std::string> override_joint_names_;

    bool enable_visual_meshes_{true};

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>
        workcell_markers_pub_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>
        workcell_collision_markers_pub_;
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::CollisionObject>>
        collision_objects_pub_;
    std::string tf_prefix_;
    std::vector<std::string> strip_flowstate_tf_prefixes_;
    std::vector<std::string> excluded_collision_namespaces_;
    std::shared_ptr<rclcpp::Service<GetResource>> get_resource_srv_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>>
        add_collision_objects_srv_;
    // Maps mesh_id to gltf binary data for a visual mesh
    absl::flat_hash_map<std::string, std::vector<uint8_t>> renderables_visual_
        ABSL_GUARDED_BY(mutex_);
    // Maps mesh_id to gltf binary data for a collision mesh
    absl::flat_hash_map<std::string, std::vector<uint8_t>>
        renderables_collision_ ABSL_GUARDED_BY(mutex_);
    // Maps mesh_id to primitive shape data
    absl::flat_hash_map<std::string, shape_msgs::msg::SolidPrimitive>
        primitives_collision_ ABSL_GUARDED_BY(mutex_);
    // Maps the tf frame of an object to multiple mesh_ids
    absl::flat_hash_map<std::string, absl::flat_hash_set<std::string>>
        collision_tf_frame_to_mesh_ids_ ABSL_GUARDED_BY(mutex_);
    // Maps a mesh_id to the pose of the mesh relative to it's tf frame
    absl::flat_hash_map<std::string, geometry_msgs::msg::Pose>
        relative_mesh_pose_ ABSL_GUARDED_BY(mutex_);
    absl::flat_hash_set<std::string> tf_frame_names_;
    std::optional<std::vector<std::string>> send_object_names_
        ABSL_GUARDED_BY(mutex_) = std::nullopt;
    bool send_new_objects_ ABSL_GUARDED_BY(mutex_) = true;
    bool shutdown_ ABSL_GUARDED_BY(mutex_) = false;
    std::shared_ptr<std::thread> viz_thread_;
    absl::Mutex mutex_;  // protects send_object_names_, send_new_objects_
    std::string mesh_url_prefix_;

    // Throttling & deadbanding for collision object pose updates
    double collision_object_update_rate_hz_{10.0};
    double collision_object_min_position_delta_{0.001};  // 1 mm
    double collision_object_min_rotation_delta_{0.01};   // ~0.57 degrees
    absl::flat_hash_map<std::string, geometry_msgs::msg::Pose>
        last_published_collision_pose_ ABSL_GUARDED_BY(mutex_);
    absl::flat_hash_map<std::string, rclcpp::Time>
        last_published_collision_time_ ABSL_GUARDED_BY(mutex_);
    ~Data();
  };
  std::shared_ptr<Data> data_;
};

}  // namespace flowstate_ros_bridge.

#endif  // MOVEIT_SCENE_BRIDGE_HPP_
