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

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "absl/log/log.h"
#include "class_loader/class_loader.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "moveit_flowstate_ros_bridge.pb.h"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

///=============================================================================
std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
PublishStaticTransforms(
    rclcpp::Node::SharedPtr node,
    const intrinsic::FlowstateRosBridgeConfig& ros_config) {
  auto static_broadcaster =
    std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

  bool publish_world_root_tf = true;
  if (ros_config.has_moveit_scene_bridge_config() &&
      ros_config.moveit_scene_bridge_config().has_publish_world_root_tf()) {
    publish_world_root_tf =
        ros_config.moveit_scene_bridge_config().publish_world_root_tf();
  }

  bool publish_robot_base_tf = true;
  if (ros_config.has_moveit_scene_bridge_config() &&
      ros_config.moveit_scene_bridge_config().has_publish_robot_base_tf()) {
    publish_robot_base_tf =
        ros_config.moveit_scene_bridge_config().publish_robot_base_tf();
  }

  std::string world_frame = "world";
  if (publish_world_root_tf) {
    // 1. world -> root
    geometry_msgs::msg::TransformStamped world_root_tf;
    world_root_tf.header.stamp = node->now();
    if (ros_config.has_moveit_scene_bridge_config() &&
        !ros_config.moveit_scene_bridge_config().world_tf_prefix().empty()) {
      world_frame = ros_config.moveit_scene_bridge_config().world_tf_prefix();
    }
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
    LOG(INFO) << "Publishing static transform: [" << world_frame << " -> root]";
  }

  std::string robot_base_frame = "ur_module/base_link";
  if (publish_robot_base_tf) {
    // 2. <robot_base_frame_id> -> base_link (180 degree yaw rotation)
    geometry_msgs::msg::TransformStamped robot_base_tf;
    robot_base_tf.header.stamp = node->now();
    if (ros_config.has_moveit_scene_bridge_config() &&
        !ros_config.moveit_scene_bridge_config().robot_base_frame_id().empty()) {
      robot_base_frame =
          ros_config.moveit_scene_bridge_config().robot_base_frame_id();
    }
    robot_base_tf.header.frame_id = robot_base_frame;
    robot_base_tf.child_frame_id = "base_link";
    robot_base_tf.transform.translation.x = 0.0;
    robot_base_tf.transform.translation.y = 0.0;
    robot_base_tf.transform.translation.z = 0.0;
    // 180 degrees yaw rotation around Z axis
    robot_base_tf.transform.rotation.x = 0.0;
    robot_base_tf.transform.rotation.y = 0.0;
    robot_base_tf.transform.rotation.z = 1.0;
    robot_base_tf.transform.rotation.w = 0.0;
    static_transforms.push_back(robot_base_tf);
    LOG(INFO) << "Publishing static transform: [" << robot_base_frame
              << " -> base_link]";
  }

  if (!static_transforms.empty()) {
    static_broadcaster->sendTransform(static_transforms);
  } else {
    LOG(INFO) << "No static transforms configured to publish.";
  }

  return static_broadcaster;
}

///=============================================================================
intrinsic_proto::config::RuntimeContext GetRuntimeContext() {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file;
  runtime_context_file.open("/etc/intrinsic/runtime_config.pb",
                            std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    // Return default context for running locally
    std::cerr << "Warning: using default RuntimeContext\n";
  }
  return runtime_context;
}

///=============================================================================
int main(int argc, char* argv[]) {
  auto runtime_context = GetRuntimeContext();
  intrinsic::FlowstateRosBridgeConfig ros_config;
  if (!runtime_context.config().UnpackTo(&ros_config)) {
    LOG(WARNING) << "Error unpacking FlowstateRosBridgeConfig from service "
                    "config file... Passing empty ros args to node";
  }

  // Determine router address (prefer external_zenoh_router_address if provided)
  std::string router_address = ros_config.external_zenoh_router_address();
  if (router_address.empty()) {
    router_address = ros_config.flowstate_zenoh_router_address();
  }
  if (!router_address.empty()) {
    std::string zenoh_config_override =
        "connect/endpoints=[\"" + router_address + "\"]";
    setenv("ZENOH_CONFIG_OVERRIDE", zenoh_config_override.c_str(), 1);
    LOG(INFO) << "ZENOH_CONFIG_OVERRIDE: " << zenoh_config_override;
  } else {
    LOG(INFO) << "No Zenoh router address configured; leaving "
                 "ZENOH_CONFIG_OVERRIDE unchanged.";
  }

  rclcpp::init(argc, argv);

  rclcpp::experimental::executors::EventsExecutor exec;
  rclcpp::NodeOptions options;

  std::vector<rclcpp::Parameter> params;
  // Get parameters from config (only emplace if set so defaults are preserved)
  if (!ros_config.executive_service_address().empty()) {
    params.emplace_back("executive_service_address",
                        ros_config.executive_service_address());
  }
  if (ros_config.executive_deadline_seconds() > 0.0) {
    params.emplace_back("executive_deadline_seconds",
                        ros_config.executive_deadline_seconds());
  }
  if (ros_config.executive_update_rate_millis() > 0) {
    params.emplace_back("executive_update_rate_millis",
                        ros_config.executive_update_rate_millis());
  }
  if (!ros_config.skill_registry_address().empty()) {
    params.emplace_back("skill_registry_address",
                        ros_config.skill_registry_address());
  }
  if (!ros_config.solution_service_address().empty()) {
    params.emplace_back("solution_service_address",
                        ros_config.solution_service_address());
  }
  if (!ros_config.world_service_address().empty()) {
    params.emplace_back("world_service_address",
                        ros_config.world_service_address());
  }
  if (!ros_config.geometry_service_address().empty()) {
    params.emplace_back("geometry_service_address",
                        ros_config.geometry_service_address());
  }
  if (!ros_config.flowstate_zenoh_router_address().empty()) {
    params.emplace_back("flowstate_zenoh_router_address",
                        ros_config.flowstate_zenoh_router_address());
  }

  std::vector<std::string> plugin_list(ros_config.bridge_plugins().begin(),
                                       ros_config.bridge_plugins().end());
  if (plugin_list.empty()) {
    plugin_list.push_back("flowstate_ros_bridge::MoveitSceneBridge");
  }
  params.emplace_back("bridge_plugins", plugin_list);

  const auto& scene_config = ros_config.moveit_scene_bridge_config();
  if (!scene_config.world_tf_prefix().empty()) {
    params.emplace_back("world_tf_prefix", scene_config.world_tf_prefix());
  }
  if (scene_config.strip_flowstate_tf_prefix_size() > 0) {
    std::vector<std::string> strip_flowstate_tf_prefix_list(
        scene_config.strip_flowstate_tf_prefix().begin(),
        scene_config.strip_flowstate_tf_prefix().end());
    params.emplace_back("strip_flowstate_tf_prefix",
                        strip_flowstate_tf_prefix_list);
  }
  std::string mesh_url_prefix = scene_config.mesh_url_prefix();
  if (mesh_url_prefix.empty()) {
    mesh_url_prefix = "http://localhost:8123/";
  }
  params.emplace_back("mesh_url_prefix", mesh_url_prefix);

  std::string collision_topic = scene_config.collision_objects_topic();
  if (collision_topic.empty()) {
    collision_topic = "/collision_object";
  }
  params.emplace_back("collision_objects_topic", collision_topic);

  std::vector<std::string> excluded_collision_namespaces_list(
      scene_config.excluded_collision_namespaces().begin(),
      scene_config.excluded_collision_namespaces().end());
  if (excluded_collision_namespaces_list.empty()) {
    excluded_collision_namespaces_list = {
        "ur_module",
        "robotiq_gripper",
        "ecat_ft_hal_module_with_adapters_with_sim",
    };
  }
  params.emplace_back("excluded_collision_namespaces",
                      excluded_collision_namespaces_list);

  params.emplace_back("enable_robot_joint_state_topic",
                      scene_config.has_enable_robot_joint_state_topic()
                          ? scene_config.enable_robot_joint_state_topic()
                          : true);

  std::string joint_state_topic = scene_config.robot_joint_state_topic();
  if (joint_state_topic.empty()) {
    joint_state_topic = "/joint_states";
  }
  params.emplace_back("robot_joint_state_topic", joint_state_topic);

  std::string base_frame_id = scene_config.robot_base_frame_id();
  if (base_frame_id.empty()) {
    base_frame_id = "ur_module/base_link";
  }
  params.emplace_back("robot_base_frame_id", base_frame_id);

  std::string controller_instance = scene_config.robot_controller_instance();
  if (controller_instance.empty()) {
    controller_instance = "icon";
  }
  params.emplace_back("robot_controller_instance", controller_instance);

  params.emplace_back("throttle_robot_state_topic",
                      scene_config.throttle_robot_state_topic());

  std::vector<std::string> override_joint_names_list(
      scene_config.override_joint_names().begin(),
      scene_config.override_joint_names().end());
  if (override_joint_names_list.empty()) {
    override_joint_names_list = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
    };
  }
  params.emplace_back("override_joint_names", override_joint_names_list);

  double update_rate_hz = scene_config.collision_objects_update_rate_hz();
  if (update_rate_hz <= 0.0) {
    update_rate_hz = 10.0;
  }
  params.emplace_back("collision_objects_update_rate_hz", update_rate_hz);

  double min_pos_delta = scene_config.collision_objects_min_position_delta();
  if (min_pos_delta <= 0.0) {
    min_pos_delta = 0.001;
  }
  params.emplace_back("collision_objects_min_position_delta", min_pos_delta);

  double min_rot_delta = scene_config.collision_objects_min_rotation_delta();
  if (min_rot_delta <= 0.0) {
    min_rot_delta = 0.01;
  }
  params.emplace_back("collision_objects_min_rotation_delta", min_rot_delta);

  params.emplace_back("publish_world_root_tf",
                      scene_config.has_publish_world_root_tf()
                          ? scene_config.publish_world_root_tf()
                          : true);
  params.emplace_back("publish_robot_base_tf",
                      scene_config.has_publish_robot_base_tf()
                          ? scene_config.publish_robot_base_tf()
                          : true);

  params.emplace_back("autostart", true);
  std::string service_tunnel = ros_config.solution_service_address();
  if (service_tunnel.empty()) {
    service_tunnel = "localhost:17080";
  }
  params.emplace_back("service_tunnel", service_tunnel);

  options.parameter_overrides(params);

  // Get namespace from config
  std::vector<std::string> remap_rules;
  remap_rules.push_back("--ros-args");
  if (ros_config.workcell_id() != "") {
    remap_rules.push_back("-r");
    remap_rules.push_back("__ns:=/" + ros_config.workcell_id());
  }
  remap_rules.push_back("-r");
  remap_rules.push_back("__node:=flowstate_ros_bridge");
  options.arguments(remap_rules);

  // Create and spin the FlowstateROSBridge node
  // Adapted from rclcpp_components::node_main.cpp.in
  std::string library_name = "libflowstate_ros_bridge_component.so";
  std::string class_name =
      "rclcpp_components::NodeFactoryTemplate<flowstate_ros_bridge::"
      "FlowstateROSBridge>";

  LOG(INFO) << "Load library " << library_name;
  auto loader = std::make_unique<class_loader::ClassLoader>(library_name);
  std::vector<std::string> classes =
      loader->getAvailableClasses<rclcpp_components::NodeFactory>();

  if (std::find(classes.begin(), classes.end(), class_name) == classes.end()) {
    LOG(INFO) << "Class " << class_name << " not found in library "
              << library_name;
    return 1;
  }
  LOG(INFO) << "Instantiate class " << class_name;
  std::shared_ptr<rclcpp_components::NodeFactory> node_factory = nullptr;
  try {
    node_factory =
        loader->createInstance<rclcpp_components::NodeFactory>(class_name);
  } catch (const std::exception& ex) {
    LOG(ERROR) << "Failed to load library " << ex.what();
    return 1;
  } catch (...) {
    LOG(ERROR) << "Failed to load library";
    return 1;
  }
  // Scope to destruct node_wrapper before shutdown
  {
    auto static_tf_node = rclcpp::Node::make_shared("bridge_static_transforms");
    auto static_broadcaster = PublishStaticTransforms(static_tf_node, ros_config);
    exec.add_node(static_tf_node->get_node_base_interface());

    rclcpp_components::NodeInstanceWrapper node_wrapper =
        node_factory->create_node_instance(options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node =
        node_wrapper.get_node_base_interface();
    exec.add_node(node);

    exec.spin();

    exec.remove_node(node_wrapper.get_node_base_interface());
    exec.remove_node(static_tf_node->get_node_base_interface());
  }

  // Shutdown ROS
  rclcpp::shutdown();

  return 0;
}
