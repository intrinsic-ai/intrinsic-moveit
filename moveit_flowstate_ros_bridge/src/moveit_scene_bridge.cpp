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

#include "moveit_scene_bridge.hpp"

#include <geometric_shapes/mesh_operations.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/strip.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/eigen.h"
#include "tf2_ros/qos.hpp"

namespace flowstate_ros_bridge {

constexpr const char* kTfPrefixParamName = "world_tf_prefix";
constexpr const char* kStripFlowstateTfPrefixParamName =
    "strip_flowstate_tf_prefix";
constexpr const char* kResourceServiceName = "flowstate_get_resource";
constexpr const char* kMeshUrlPrefixParamName = "mesh_url_prefix";
constexpr const char* kCollisionObjectsTopicParamName =
    "collision_objects_topic";
constexpr const char* kExcludedCollisionNamespacesParamName =
    "excluded_collision_namespaces";
constexpr const char* kEnableRobotJointStateTopicParamName =
    "enable_robot_joint_state_topic";
constexpr const char* kRobotJointStateTopicParamName =
    "robot_joint_state_topic";
constexpr const char* kRobotBaseFrameIDParamName = "robot_base_frame_id";
constexpr const char* kRobotControllerNameParamName =
    "robot_controller_instance";
constexpr const char* kThrottleRobotStateTopicParamName =
    "throttle_robot_state_topic";
constexpr const char* kOverrideJointNamesParamName = "override_joint_names";
constexpr const char* kCollisionObjectsUpdateRateHzParamName =
    "collision_objects_update_rate_hz";
constexpr const char* kCollisionObjectsMinPositionDeltaParamName =
    "collision_objects_min_position_delta";
constexpr const char* kCollisionObjectsMinRotationDeltaParamName =
    "collision_objects_min_rotation_delta";
constexpr const char* kPublishWorldRootTfParamName = "publish_world_root_tf";
constexpr const char* kPublishRobotBaseTfParamName = "publish_robot_base_tf";
constexpr const char* kEnableVisualMeshesParamName = "enable_visual_meshes";

///=============================================================================
void MoveitSceneBridge::declare_ros_parameters(
    ROSNodeInterfaces ros_node_interfaces) {
  const auto& param_interface =
      ros_node_interfaces
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  param_interface->declare_parameter(kTfPrefixParamName,
                                     rclcpp::ParameterValue{""});
  param_interface->declare_parameter(
      kStripFlowstateTfPrefixParamName,
      rclcpp::ParameterValue(std::vector<std::string>{}));
  param_interface->declare_parameter(
      kMeshUrlPrefixParamName,
      rclcpp::ParameterValue{"http://localhost:8123/"});
  param_interface->declare_parameter(
      kCollisionObjectsTopicParamName,
      rclcpp::ParameterValue("/collision_object"));
  param_interface->declare_parameter(
      kExcludedCollisionNamespacesParamName,
      rclcpp::ParameterValue(std::vector<std::string>{"ur3e_robot"}));
  param_interface->declare_parameter(kEnableRobotJointStateTopicParamName,
                                     rclcpp::ParameterValue(true));
  param_interface->declare_parameter(kRobotJointStateTopicParamName,
                                     rclcpp::ParameterValue("/joint_states"));
  param_interface->declare_parameter(
      kRobotBaseFrameIDParamName,
      rclcpp::ParameterValue("ur_module/base_link"));
  param_interface->declare_parameter(
      kRobotControllerNameParamName,
      rclcpp::ParameterValue("icon"));
  param_interface->declare_parameter(kThrottleRobotStateTopicParamName,
                                     rclcpp::ParameterValue(false));
  param_interface->declare_parameter(
      kOverrideJointNamesParamName,
      rclcpp::ParameterValue(std::vector<std::string>{}));
  param_interface->declare_parameter(
      kCollisionObjectsUpdateRateHzParamName,
      rclcpp::ParameterValue(10.0));
  param_interface->declare_parameter(
      kCollisionObjectsMinPositionDeltaParamName,
      rclcpp::ParameterValue(0.001));
  param_interface->declare_parameter(
      kCollisionObjectsMinRotationDeltaParamName,
      rclcpp::ParameterValue(0.01));
  param_interface->declare_parameter(kPublishWorldRootTfParamName,
                                     rclcpp::ParameterValue(true));
  param_interface->declare_parameter(kPublishRobotBaseTfParamName,
                                     rclcpp::ParameterValue(true));
  param_interface->declare_parameter(kEnableVisualMeshesParamName,
                                     rclcpp::ParameterValue(true));
}

///=============================================================================
bool MoveitSceneBridge::initialize(
    ROSNodeInterfaces ros_node_interfaces,
    std::shared_ptr<Executive> /*executive_client*/,
    std::shared_ptr<World> world_client) {
  data_ = std::make_shared<Data>();
  data_->node_interfaces_ = std::move(ros_node_interfaces);
  data_->world_ = std::move(world_client);

  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>
      param_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeParametersInterface>();

  data_->get_resource_srv_ = rclcpp::create_service<GetResource>(
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>(),
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeServicesInterface>(),
      kResourceServiceName,
      [data_ = this->data_](const std::shared_ptr<GetResource::Request> request,
                            std::shared_ptr<GetResource::Response> response) {
        const std::string gltf_id = request->path;
        LOG(INFO) << "request resource path: " << gltf_id;
        absl::MutexLock lock(&data_->mutex_);
        if (auto it = data_->renderables_visual_.find(gltf_id);
            it != data_->renderables_visual_.end()) {
          response->status_code = GetResource::Response::OK;
          response->body = it->second;
        } else if (auto it_col = data_->renderables_collision_.find(gltf_id);
                   it_col != data_->renderables_collision_.end()) {
          response->status_code = GetResource::Response::OK;
          response->body = it_col->second;
        } else {
          response->status_code = GetResource::Response::ERROR;
          return;
        }
      },
      rclcpp::ServicesQoS(), nullptr);

  data_->add_collision_objects_srv_ = rclcpp::create_service<
      std_srvs::srv::Trigger>(
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>(),
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeServicesInterface>(),
      "~/add_collision_objects",
      [data_ = this->data_](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        data_->addCollisionObjectsToScene();
        response->success = true;
        response->message = "Collision objects added to scene";
      },
      rclcpp::ServicesQoS(), nullptr);

  data_->tf_prefix_ = param_interface->get_parameter(kTfPrefixParamName)
                          .get_value<std::string>();
  data_->strip_flowstate_tf_prefixes_ =
      param_interface->get_parameter(kStripFlowstateTfPrefixParamName)
          .as_string_array();
  data_->excluded_collision_namespaces_ =
      param_interface->get_parameter(kExcludedCollisionNamespacesParamName)
          .as_string_array();
  data_->override_joint_names_ =
      param_interface->get_parameter(kOverrideJointNamesParamName)
          .as_string_array();
  data_->collision_object_update_rate_hz_ =
      param_interface->get_parameter(kCollisionObjectsUpdateRateHzParamName)
          .as_double();
  data_->collision_object_min_position_delta_ =
      param_interface
          ->get_parameter(kCollisionObjectsMinPositionDeltaParamName)
          .as_double();
  data_->collision_object_min_rotation_delta_ =
      param_interface
          ->get_parameter(kCollisionObjectsMinRotationDeltaParamName)
          .as_double();
  data_->enable_visual_meshes_ =
      param_interface->get_parameter(kEnableVisualMeshesParamName).as_bool();

  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>
      topics_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeTopicsInterface>();

  data_->tf_pub_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(
      param_interface, topics_interface, "tf",
      tf2_ros::DynamicBroadcasterQoS());

  const rclcpp::QoS markers_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
  data_->workcell_markers_pub_ =
      rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
          param_interface, topics_interface, "workcell_markers/visual",
          markers_qos);
  data_->workcell_collision_markers_pub_ =
      rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
          param_interface, topics_interface, "workcell_markers/collision",
          markers_qos);
  data_->collision_objects_pub_ =
      rclcpp::create_publisher<moveit_msgs::msg::CollisionObject>(
          param_interface, topics_interface,
          param_interface->get_parameter(kCollisionObjectsTopicParamName)
              .as_string(),
          rclcpp::ServicesQoS());

  data_->mesh_url_prefix_ =
      param_interface->get_parameter(kMeshUrlPrefixParamName)
          .get_value<std::string>();

  auto tf_sub_ = data_->world_->CreateTfSubscription(
      [this](const intrinsic_proto::TFMessage& msg) { this->TfCallback(msg); });
  if (!tf_sub_.ok()) {
    LOG(ERROR) << "Unable to create TF Subscription: " << tf_sub_.status();
    return false;
  }
  LOG(INFO) << "Subscribed to Flowstate TF topic";
  data_->tf_sub_ = std::move(*tf_sub_);

  // Robot States Bridge
  data_->robot_joint_state_topic_enabled_ =
      param_interface->get_parameter(kEnableRobotJointStateTopicParamName)
          .as_bool();
  LOG(INFO) << "Robot Joint State Bridge Enabled: "
            << data_->robot_joint_state_topic_enabled_;

  // Create ROS publishers
  data_->robot_joint_state_pub_ =
      rclcpp::create_publisher<sensor_msgs::msg::JointState>(
          param_interface, topics_interface,
          param_interface->get_parameter(kRobotJointStateTopicParamName)
              .as_string(),
          rclcpp::SensorDataQoS());

  // Create Flowstate subscriptions
  std::string robot_controller_instance =
      param_interface->get_parameter(kRobotControllerNameParamName).as_string();
  bool throttle_robot_state_topic =
      param_interface->get_parameter(kThrottleRobotStateTopicParamName)
          .as_bool();
  auto robot_state_sub = data_->world_->CreateRobotStateSubscription(
      [this](const intrinsic_proto::data_logger::LogItem& msg) {
        this->RobotStateCallback(msg);
      },
      robot_controller_instance, throttle_robot_state_topic);
  if (!robot_state_sub.ok()) {
    LOG(ERROR) << "Unable to create Robot State Subscription: "
               << robot_state_sub.status();
    return false;
  }
  LOG(INFO) << "Subscribed to Flowstate Robot State topic";
  data_->robot_state_sub_ = std::move(*robot_state_sub);

  data_->robot_base_frame_id_ =
      param_interface->get_parameter(kRobotBaseFrameIDParamName).as_string();

  // Start a thread to publish sceneObject visualization messages whenever a new
  // object arrives
  std::weak_ptr<Data> data_wp = data_;
  data_->viz_thread_ = std::make_shared<std::thread>([data_wp]() {
    while (rclcpp::ok()) {
      if (auto data = data_wp.lock()) {
        std::optional<std::vector<std::string>> send_object_names;
        {
          absl::MutexLock lock(&data->mutex_,
                               absl::Condition(
                                   +[](Data* d) { return d->send_new_objects_ || d->shutdown_; },
                                   data.get()));
          if (data->shutdown_) {
            break;
          }
          send_object_names = data->send_object_names_;
          if (data->send_object_names_.has_value()) {
            data->send_object_names_.value().clear();
          }
          data->send_new_objects_ = false;
        }

        const absl::Status status =
            data->SendObjectVisualizationMessages(send_object_names);
        if (!status.ok()) {
          LOG(ERROR) << "Unable to send object visualization messages: "
                     << status.message();
          continue;
        }
      } else {
        LOG(ERROR) << "data has expired! Terminating thread sending "
                      "visualization objects";
        return;
      }
    }
  });

  return true;
}

absl::Status MoveitSceneBridge::Data::SendObjectVisualizationMessages(
    std::optional<std::vector<std::string>> object_names) {
  absl::StatusOr<std::vector<intrinsic::world::WorldObject>> objects =
      world_->GetObjects(std::move(object_names));
  if (!objects.ok()) {
    return objects.status();
  }

  size_t total_visual_gltf_size = 0, total_collision_gltf_size = 0;
  visualization_msgs::msg::MarkerArray visual_array_msg, collision_array_msg;
  rclcpp::Clock clock;
  const rclcpp::Time t_now = clock.now();
  bool new_collision_mesh = false;
  std::vector<std::string> new_collision_object_frames;

  for (const intrinsic::world::WorldObject& object : *objects) {
    int object_id = 0;
    const intrinsic_proto::world::Object& proto = object.Proto();
    for (const auto& entity : proto.entities()) {
      if (!entity.second.has_geometry_component()) {
        continue;
      }
      for (const auto& named_geometry :
           entity.second.geometry_component().named_geometries()) {
        if (!enable_visual_meshes_ && named_geometry.first == "Intrinsic_Visual") {
          continue;
        }

        if (named_geometry.first != "Intrinsic_Visual" &&
            named_geometry.first != "Intrinsic_Collision") {
          continue;
        }

        const auto& named_geoms = named_geometry.second.named_geometries();

        for (const auto& [inner_name, transformed_geometry] :
             named_geoms) {  // map<string,
                             // intrinsic_proto.geometry.v1.TransformedGeometry>
          const std::string object_name = StripTfPrefixes(
              object.Name().value(), strip_flowstate_tf_prefixes_);
          const std::vector<absl::string_view> parts =
              absl::StrSplit(object_name, '/');
          const absl::string_view short_name = parts.back();

          // Exclude a collision geometry if its namespace is within
          // excluded_collision_namespaces_
          auto should_exclude_collision = [&]() {
            if (named_geometry.first == "Intrinsic_Collision") {
              for (const auto& ns : excluded_collision_namespaces_) {
                if (short_name == ns) {
                  return true;
                }
              }
            }
            return false;
          };
          if (should_exclude_collision()) {
            continue;
          }

          // In V1 SDK, the Flowstate TF stream appends the short object name
          // again between the full path and the entity name.
          const std::string tf_frame_name = absl::StrFormat(
              "%s%s/%s", tf_prefix_.c_str(), short_name, entity.second.name());
          // A unique identifier for the mesh. For visual meshes, it is the
          // gltf_path. For collision meshes, it is the tf_frame_name of the
          // object
          std::string visual_mesh_id;
          std::string collision_mesh_id = std::string("/") + tf_frame_name +
                                          std::string("/") +
                                          std::string(inner_name);

          visualization_msgs::msg::Marker marker_msg;
          marker_msg.header.frame_id = tf_frame_name;
          marker_msg.header.stamp = t_now;
          marker_msg.ns = tf_frame_name;
          marker_msg.action = visualization_msgs::msg::Marker::ADD;
          // Leave color as (1, 1, 1, 1) so that the mesh color is as expected
          // from the embedded glTF textures.
          marker_msg.color.r = 1.0;
          marker_msg.color.g = 1.0;
          marker_msg.color.b = 1.0;
          marker_msg.color.a = 0.25;
          // Set lifetime to (0, 0) to indicate these meshes never expire.
          marker_msg.lifetime.sec = 0;
          marker_msg.lifetime.nanosec = 0;
          // Lock the mesh to its TF frame so that motion is handled correctly
          marker_msg.frame_locked = true;

          const auto& geometry = transformed_geometry.geometry();
          // the geometry will either have 'inline_geometry_data' (primitive
          // meshes) or 'geo_ref' (gltf data)
          if (geometry.has_geo_ref()) {
            const auto& geo_ref = geometry.geo_ref();

            // Let's be smarter in the future. For now, just skip over
            // the intcas:// prefix
            const std::string gltf_path = absl::StrFormat(
                "gltf/%s_%s.glb", geo_ref.exact_geometry_ref().substr(9),
                geo_ref.renderable_ref().substr(9));
            visual_mesh_id = std::string("/") + gltf_path;

            auto& target_renderables =
                (named_geometry.first == "Intrinsic_Visual")
                    ? renderables_visual_
                    : renderables_collision_;
            const std::string& target_mesh_id =
                (named_geometry.first == "Intrinsic_Visual")
                    ? visual_mesh_id
                    : collision_mesh_id;
            bool should_fetch_mesh =
                !target_renderables.contains(target_mesh_id);

            if (should_fetch_mesh) {
              const absl::StatusOr<std::string> gltf = world_->GetGltf(
                  geo_ref.exact_geometry_ref(), geo_ref.renderable_ref());
              if (!gltf.ok()) {
                LOG(ERROR) << "Unable to fetch renderable for " << tf_frame_name
                           << ": " << gltf.status();
                continue;
              }
              if (named_geometry.first == "Intrinsic_Visual") {
                total_visual_gltf_size += gltf->size();
              } else {
                total_collision_gltf_size += gltf->size();
              }
              std::vector<uint8_t> gltf_data;
              gltf_data.resize(gltf->size());
              memcpy(&gltf_data[0], gltf->data(), gltf->size());

              LOG(INFO) << "Fetched " << gltf->size() << " bytes for "
                        << tf_frame_name;

              absl::MutexLock lock(&mutex_);
              target_renderables.emplace(target_mesh_id, std::move(gltf_data));
            }

            // Currently all of our meshes have unit scaling. If this changes,
            // we could use Transform::computeRotationScaling() but that costs
            // a SVD, and it doesn't seem worth it when _all_ of our meshes are
            // already at unit scale.
            marker_msg.scale.x = 1.0;
            marker_msg.scale.y = 1.0;
            marker_msg.scale.z = 1.0;
            marker_msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            // Set the mesh resource path to the HTTP/rmw_zenoh proxy
            marker_msg.mesh_resource = mesh_url_prefix_ + gltf_path;
            marker_msg.mesh_use_embedded_materials = true;
          } else if (geometry.has_inline_geometry_data()) {
            const auto& inline_geometry = geometry.inline_geometry_data();
            if (inline_geometry.has_exact_geometry()) {
              const auto& exact_geometry =
                  inline_geometry.exact_geometry();  // ExactGeometry
              if (exact_geometry.has_primitive_set()) {
                const auto& primitive_set = exact_geometry.primitive_set();
                // Iterate through each primitive shape
                for (int i = 0; i < primitive_set.primitives_size(); ++i) {
                  const auto& transformed_primitive =
                      primitive_set.primitives(i);
                  if (transformed_primitive.has_shape()) {
                    const auto& primitive_shape = transformed_primitive.shape();

                    shape_msgs::msg::SolidPrimitive solid_prim;
                    if (primitive_shape.has_box()) {
                      marker_msg.type = visualization_msgs::msg::Marker::CUBE;
                      marker_msg.scale.x = primitive_shape.box().size().x();
                      marker_msg.scale.y = primitive_shape.box().size().y();
                      marker_msg.scale.z = primitive_shape.box().size().z();

                      solid_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
                      solid_prim.dimensions.resize(3);
                      solid_prim
                          .dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] =
                          primitive_shape.box().size().x();
                      solid_prim
                          .dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] =
                          primitive_shape.box().size().y();
                      solid_prim
                          .dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] =
                          primitive_shape.box().size().z();
                    } else if (primitive_shape.has_cylinder()) {
                      marker_msg.type =
                          visualization_msgs::msg::Marker::CYLINDER;
                      marker_msg.scale.x =
                          primitive_shape.cylinder().radius() * 2.0;
                      marker_msg.scale.y =
                          primitive_shape.cylinder().radius() * 2.0;
                      marker_msg.scale.z = primitive_shape.cylinder().length();

                      solid_prim.type =
                          shape_msgs::msg::SolidPrimitive::CYLINDER;
                      solid_prim.dimensions.resize(2);
                      solid_prim.dimensions
                          [shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
                          primitive_shape.cylinder().length();
                      solid_prim.dimensions
                          [shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
                          primitive_shape.cylinder().radius();
                    } else if (primitive_shape.has_sphere()) {
                      marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
                      marker_msg.scale.x =
                          primitive_shape.sphere().radius() * 2.0;
                      marker_msg.scale.y =
                          primitive_shape.sphere().radius() * 2.0;
                      marker_msg.scale.z =
                          primitive_shape.sphere().radius() * 2.0;

                      solid_prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
                      solid_prim.dimensions.resize(1);
                      solid_prim.dimensions
                          [shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] =
                          primitive_shape.sphere().radius();
                    } else if (primitive_shape.has_capsule()) {
                      marker_msg.type =
                          visualization_msgs::msg::Marker::CYLINDER;
                      marker_msg.scale.x =
                          primitive_shape.capsule().radius() * 2.0;
                      marker_msg.scale.y =
                          primitive_shape.capsule().radius() * 2.0;
                      marker_msg.scale.z = primitive_shape.capsule().length();

                      solid_prim.type =
                          shape_msgs::msg::SolidPrimitive::CYLINDER;
                      solid_prim.dimensions.resize(2);
                      solid_prim.dimensions
                          [shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
                          primitive_shape.capsule().length();
                      solid_prim.dimensions
                          [shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
                          primitive_shape.capsule().radius();
                    } else if (primitive_shape.has_ellipsoid()) {
                      marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
                      marker_msg.scale.x =
                          primitive_shape.ellipsoid().radii().x() * 2.0;
                      marker_msg.scale.y =
                          primitive_shape.ellipsoid().radii().y() * 2.0;
                      marker_msg.scale.z =
                          primitive_shape.ellipsoid().radii().z() * 2.0;

                      solid_prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
                      solid_prim.dimensions.resize(1);
                      solid_prim.dimensions
                          [shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] =
                          std::max({primitive_shape.ellipsoid().radii().x(),
                                    primitive_shape.ellipsoid().radii().y(),
                                    primitive_shape.ellipsoid().radii().z()});
                    } else if (primitive_shape.has_frustum()) {
                      LOG(WARNING) << "Found a Frustum primitive. Unsupported "
                                      "for visualization.";
                      continue;
                    } else {
                      LOG(WARNING) << "Found an unsupported primitive shape. "
                                      "Unsupported for visualization.";
                      continue;
                    }

                    {
                      absl::MutexLock lock(&mutex_);
                      primitives_collision_[collision_mesh_id] = solid_prim;
                    }
                  }
                }
              }
            }
          } else {
            LOG(ERROR) << "No geometry data for object " << tf_frame_name;
            continue;
          }

          // Retrieve the mesh transform from the proto
          const auto& ref_t_shape = transformed_geometry.ref_t_shape();
          if (!ref_t_shape.has_matrix4d()) {
            LOG(ERROR) << "Geometry with TF frame '" << tf_frame_name
                       << "' does not have a valid transform.";
            continue;
          }
          const absl::StatusOr<intrinsic::eigenmath::MatrixXd> transform_xd =
              intrinsic_proto::FromProto(ref_t_shape.matrix4d());
          if (!transform_xd.ok()) {
            LOG(ERROR) << "Failed to convert transform to Eigen types for "
                       << tf_frame_name << ": " << transform_xd.status();
            continue;
          }
          const intrinsic::eigenmath::Matrix4d transform_4d = *transform_xd;
          const intrinsic::eigenmath::AffineTransform3d affine(transform_4d);

          marker_msg.pose.position.x = affine.translation().x();
          marker_msg.pose.position.y = affine.translation().y();
          marker_msg.pose.position.z = affine.translation().z();
          const intrinsic::eigenmath::Quaterniond quat(affine.rotation());
          marker_msg.pose.orientation.x = quat.x();
          marker_msg.pose.orientation.y = quat.y();
          marker_msg.pose.orientation.z = quat.z();
          marker_msg.pose.orientation.w = quat.w();

          marker_msg.id = object_id++;

          if (named_geometry.first == "Intrinsic_Visual") {
            visual_array_msg.markers.push_back(marker_msg);
          } else {  // "Intrinsic_Collision"
            collision_array_msg.markers.push_back(marker_msg);
            {
              absl::MutexLock lock(&mutex_);
              relative_mesh_pose_[collision_mesh_id] = marker_msg.pose;

              if (!collision_tf_frame_to_mesh_ids_.contains(tf_frame_name)) {
                new_collision_mesh = true;
                new_collision_object_frames.push_back(tf_frame_name);
              }
              collision_tf_frame_to_mesh_ids_[tf_frame_name].insert(
                  collision_mesh_id);
            }
          }
        }
      }
    }
  }

  if (enable_visual_meshes_ && !visual_array_msg.markers.empty()) {
    workcell_markers_pub_->publish(visual_array_msg);
  }
  if (!collision_array_msg.markers.empty()) {
    workcell_collision_markers_pub_->publish(collision_array_msg);
  }

  if (new_collision_mesh) {
    addCollisionObjectsToScene(new_collision_object_frames);
  }

  LOG(INFO) << "Size of total visual gltf meshes fetched: "
            << (total_visual_gltf_size / (1024.0 * 1024.0)) << " MB";
  LOG(INFO) << "Size of total collision gltf meshes fetched: "
            << (total_collision_gltf_size / (1024.0 * 1024.0)) << " MB";

  return absl::OkStatus();
}

void MoveitSceneBridge::Data::addCollisionObjectsToScene(
    std::optional<std::vector<std::string>> object_frames) {
  rclcpp::Time t_now =
      node_interfaces_.get<rclcpp::node_interfaces::NodeClockInterface>()
          ->get_clock()
          ->now();

  absl::MutexLock lock(&mutex_);

  std::vector<std::string> object_frames_to_add;
  if (!object_frames.has_value()) {
    for (const auto& [tf_frame_name, mesh_ids] :
         collision_tf_frame_to_mesh_ids_) {
      object_frames_to_add.push_back(tf_frame_name);
    }
  } else {
    object_frames_to_add = object_frames.value();
  }

  for (const auto& tf_frame_name : object_frames_to_add) {
    auto it_frame = collision_tf_frame_to_mesh_ids_.find(tf_frame_name);
    if (it_frame == collision_tf_frame_to_mesh_ids_.end()) {
      continue;
    }
    const auto& mesh_ids = it_frame->second;
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.header.stamp = t_now;
    collision_obj.header.frame_id = tf_frame_name;
    collision_obj.id = tf_frame_name;
    collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_obj.pose.orientation.w = 1.0;

    for (const auto& mesh_id : mesh_ids) {
      const auto& relative_pose = relative_mesh_pose_[mesh_id];

      // If a renderable does not exist in renderables_collision_, use the
      // primitive shapes from primitives_collision_
      if (auto it_mesh = renderables_collision_.find(mesh_id);
          it_mesh != renderables_collision_.end()) {
        // Retrieve gltf binary data and convert it to a shape_msgs::msg::Mesh
        // message
        const auto& gltf_data = it_mesh->second;
        std::unique_ptr<shapes::Mesh> shape_mesh(shapes::createMeshFromBinary(
            reinterpret_cast<const char*>(gltf_data.data()), gltf_data.size(),
            "glb"));
        if (shape_mesh) {
          shape_msgs::msg::Mesh mesh_msg;
          mesh_msg.vertices.resize(shape_mesh->vertex_count);
          for (unsigned int i = 0; i < shape_mesh->vertex_count; ++i) {
            mesh_msg.vertices[i].x = shape_mesh->vertices[3 * i];
            mesh_msg.vertices[i].y = shape_mesh->vertices[3 * i + 1];
            mesh_msg.vertices[i].z = shape_mesh->vertices[3 * i + 2];
          }
          mesh_msg.triangles.resize(shape_mesh->triangle_count);
          for (unsigned int i = 0; i < shape_mesh->triangle_count; ++i) {
            mesh_msg.triangles[i].vertex_indices[0] =
                shape_mesh->triangles[3 * i];
            mesh_msg.triangles[i].vertex_indices[1] =
                shape_mesh->triangles[3 * i + 1];
            mesh_msg.triangles[i].vertex_indices[2] =
                shape_mesh->triangles[3 * i + 2];
          }
          collision_obj.meshes.push_back(mesh_msg);
          collision_obj.mesh_poses.push_back(relative_pose);

        } else {
          LOG(ERROR) << "Failed to convert gltf mesh for object with tf_frame "
                     << tf_frame_name;
        }
      } else {
        // Add primitive shapes and their relative pose from their TF frame
        collision_obj.primitives.push_back(primitives_collision_[mesh_id]);
        collision_obj.primitive_poses.push_back(relative_pose);
      }
    }

    LOG(INFO) << "Added collision object with tf_frame_name: '" << tf_frame_name
              << "'";
    collision_objects_pub_->publish(collision_obj);
  }
}

void MoveitSceneBridge::Data::removeCollisionObject(
    const std::string& tf_frame_name) {
  rclcpp::Time t_now =
      node_interfaces_.get<rclcpp::node_interfaces::NodeClockInterface>()
          ->get_clock()
          ->now();

  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.stamp = t_now;
  collision_obj.id = tf_frame_name;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  {
    absl::MutexLock lock(&mutex_);
    collision_tf_frame_to_mesh_ids_.erase(tf_frame_name);
    last_published_collision_pose_.erase(tf_frame_name);
    last_published_collision_time_.erase(tf_frame_name);
  }

  collision_objects_pub_->publish(collision_obj);
  LOG(INFO) << "Removed collision object with tf_frame_name: '" << tf_frame_name
            << "'";
}

void MoveitSceneBridge::Data::updateCollisionObjects(
    const std::vector<geometry_msgs::msg::TransformStamped>& transforms) {
  rclcpp::Time t_now =
      node_interfaces_.get<rclcpp::node_interfaces::NodeClockInterface>()
          ->get_clock()
          ->now();

  absl::MutexLock lock(&mutex_);
  for (const auto& [tf_frame_name, mesh_ids] :
       collision_tf_frame_to_mesh_ids_) {
    for (const auto& ts_ros : transforms) {
      // The collision object's frame matches the child frame of the transform
      if (ts_ros.child_frame_id == tf_frame_name) {
        geometry_msgs::msg::Pose new_pose;
        new_pose.position.x = ts_ros.transform.translation.x;
        new_pose.position.y = ts_ros.transform.translation.y;
        new_pose.position.z = ts_ros.transform.translation.z;
        new_pose.orientation = ts_ros.transform.rotation;

        // 1. Rate limiting check
        if (auto it_time = last_published_collision_time_.find(tf_frame_name);
            it_time != last_published_collision_time_.end()) {
          if (collision_object_update_rate_hz_ > 0.0) {
            const double min_period = 1.0 / collision_object_update_rate_hz_;
            if ((t_now - it_time->second).seconds() < min_period) {
              break;
            }
          }
        }

        // 2. Deadbanding check (pose delta threshold)
        if (auto it_pose = last_published_collision_pose_.find(tf_frame_name);
            it_pose != last_published_collision_pose_.end()) {
          const auto& old_pose = it_pose->second;
          const double dx = new_pose.position.x - old_pose.position.x;
          const double dy = new_pose.position.y - old_pose.position.y;
          const double dz = new_pose.position.z - old_pose.position.z;
          const double pos_delta = std::sqrt(dx * dx + dy * dy + dz * dz);

          const auto& q1 = old_pose.orientation;
          const auto& q2 = new_pose.orientation;
          const double dot = std::min(
              1.0, std::abs(q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z));
          const double rot_delta = 2.0 * std::acos(dot);

          if (pos_delta < collision_object_min_position_delta_ &&
              rot_delta < collision_object_min_rotation_delta_) {
            break;
          }
        }

        moveit_msgs::msg::CollisionObject collision_obj;
        collision_obj.operation = moveit_msgs::msg::CollisionObject::MOVE;
        collision_obj.header.stamp = t_now;
        // The reference frame for the new pose should be the parent frame of
        // the transform. MoveIt calculates: world_to_object =
        // getFrameTransform(header.frame_id) * pose So world_to_child =
        // world_to_parent * parent_to_child.
        collision_obj.header.frame_id = ts_ros.header.frame_id;
        collision_obj.id = tf_frame_name;
        collision_obj.pose = new_pose;

        for (const auto& mesh_id : mesh_ids) {
          const auto& relative_pose = relative_mesh_pose_[mesh_id];
          if (renderables_collision_.contains(mesh_id)) {
            collision_obj.mesh_poses.push_back(relative_pose);
          } else {  // Primitive shapes
            collision_obj.primitive_poses.push_back(relative_pose);
          }
        }

        collision_objects_pub_->publish(collision_obj);
        last_published_collision_pose_[tf_frame_name] = new_pose;
        last_published_collision_time_[tf_frame_name] = t_now;
        break;
      }
    }
  }
}

///=============================================================================
MoveitSceneBridge::Data::~Data() {}

///=============================================================================
std::string MoveitSceneBridge::StripTfPrefixes(
    absl::string_view frame, const std::vector<std::string>& prefixes) {
  absl::string_view stripped = frame;
  for (const auto& prefix : prefixes) {
    // Add check to only strip the prefix once per frame
    if (!prefix.empty() && absl::StartsWith(stripped, prefix)) {
      stripped = absl::StripPrefix(stripped, prefix);
      break;
    }
  }
  return std::string(stripped);
}

///=============================================================================
void MoveitSceneBridge::TfCallback(const intrinsic_proto::TFMessage& tf_proto) {
  rclcpp::Clock clock;
  const rclcpp::Time t_start = clock.now();

  absl::flat_hash_set<std::string> new_tf_frame_names;
  absl::flat_hash_set<std::string> new_object_names;

  tf2_msgs::msg::TFMessage tf_ros;
  tf_ros.transforms = std::vector<geometry_msgs::msg::TransformStamped>(
      tf_proto.transforms_size());
  int tf_idx = 0;
  for (const auto& ts_proto : tf_proto.transforms()) {
    geometry_msgs::msg::TransformStamped* ts_ros = &tf_ros.transforms[tf_idx++];
    ts_ros->header.stamp = t_start;

    // Strip away Flowstate TF prefixes
    const std::string frame_id = StripTfPrefixes(
        ts_proto.header().frame_id(), data_->strip_flowstate_tf_prefixes_);

    const std::string child_frame_id = StripTfPrefixes(
        ts_proto.child_frame_id(), data_->strip_flowstate_tf_prefixes_);

    ts_ros->header.frame_id = data_->tf_prefix_ + frame_id;
    ts_ros->child_frame_id = data_->tf_prefix_ + child_frame_id;

    new_tf_frame_names.insert(ts_ros->child_frame_id);
    if (!data_->tf_frame_names_.contains(ts_ros->child_frame_id)) {
      // We parse the "OBJECT_NAME/ENTITY_NAME" string to get the OBJECT_NAME
      LOG(INFO) << "new child_frame_id: " << ts_ros->child_frame_id;
      const std::size_t str_end = child_frame_id.find('/');
      new_object_names.insert(child_frame_id.substr(0, str_end));
    }
    // The auto-generated CDR types do not currently have assignment operators
    // or helper conversion functions from the corresponding protos, so we need
    // to explicitly copy all the fields.
    const auto& t = ts_proto.transform();  // just to save some typing
    ts_ros->transform.translation.x = t.translation().x();
    ts_ros->transform.translation.y = t.translation().y();
    ts_ros->transform.translation.z = t.translation().z();
    ts_ros->transform.rotation.x = t.rotation().x();
    ts_ros->transform.rotation.y = t.rotation().y();
    ts_ros->transform.rotation.z = t.rotation().z();
    ts_ros->transform.rotation.w = t.rotation().w();
  }
  data_->tf_pub_->publish(tf_ros);

  // print a timing snapshot every 500 messages
  const rclcpp::Duration elapsed = clock.now() - t_start;
  LOG_EVERY_N(INFO, 500) << absl::StrFormat(
      "Time taken to process TF messages: %.3f ms", 1000.0 * elapsed.seconds());

  std::vector<std::string> deleted_tf_frames;
  for (const auto& tf_frame : data_->tf_frame_names_) {
    if (!new_tf_frame_names.contains(tf_frame)) {
      deleted_tf_frames.push_back(tf_frame);
    }
  }
  if (!deleted_tf_frames.empty()) {
    visualization_msgs::msg::MarkerArray array_msg;
    for (const auto& tf_frame : deleted_tf_frames) {
      data_->removeCollisionObject(tf_frame);

      LOG(INFO) << "Removed sceneObject with frame_id " << tf_frame
                << " in the world, updating visualization markers.";

      visualization_msgs::msg::Marker marker_msg;
      marker_msg.ns = tf_frame;
      marker_msg.action = visualization_msgs::msg::Marker::DELETEALL;

      array_msg.markers.push_back(std::move(marker_msg));
    }
    if (data_->enable_visual_meshes_) {
      data_->workcell_markers_pub_->publish(array_msg);
    }
    data_->workcell_collision_markers_pub_->publish(array_msg);
  }

  if (!new_object_names.empty()) {
    {
      absl::MutexLock lock(&data_->mutex_);
      if (data_->send_object_names_.has_value()) {
        data_->send_object_names_.value().insert(
            data_->send_object_names_.value().end(), new_object_names.begin(),
            new_object_names.end());
      } else {
        data_->send_object_names_ = std::vector<std::string>(
            new_object_names.begin(), new_object_names.end());
      }
      // Signal background thread to send object visualization messages
      data_->send_new_objects_ = true;
    }
  }

  data_->tf_frame_names_ = std::move(new_tf_frame_names);

  data_->updateCollisionObjects(tf_ros.transforms);
}

///=============================================================================
void MoveitSceneBridge::RobotStateCallback(
    const intrinsic_proto::data_logger::LogItem& log_item) {
  rclcpp::Clock clock;
  const rclcpp::Time t_start = clock.now();
  const auto& payload = log_item.payload();

  switch (payload.data_case()) {
    case intrinsic_proto::data_logger::LogItem::Payload::kIconRobotStatus: {
      if (data_->robot_joint_state_topic_enabled_) {
        HandleRobotStatus(payload.icon_robot_status(), t_start);
      }
      break;
    }

    default: {
      std::string msg;
      const auto* descriptor = payload.GetDescriptor();
      const auto* field = descriptor->FindFieldByNumber(payload.data_case());
      if (field) {
        msg = absl::StrFormat("Received unhandled data type: %s (ID: %d)",
                              field->name(), payload.data_case());
      } else {
        msg = absl::StrFormat("Received unknown or unset data type (ID: %d)",
                              payload.data_case());
      }
      LOG_EVERY_N(INFO, 100) << msg;
      break;
    }
  }

  // print the translation time every 5000 messages
  const rclcpp::Duration elapsed = clock.now() - t_start;
  LOG_EVERY_N(INFO, 5000) << absl::StrFormat(
      "Robot state translation time: %.3f ms", 1000.0 * elapsed.seconds());
}

void MoveitSceneBridge::HandleRobotStatus(
    const intrinsic_proto::icon::RobotStatus& robot_status,
    const rclcpp::Time& time) {
  // On first execution, cache the part names to avoid repeated map iteration
  if (!data_->robot_arm_part_name_.has_value()) {
    for (const auto& entry : robot_status.status_map()) {
      const std::string& part_name = entry.first;
      const auto& part_status = entry.second;
      if (!part_status.joint_states().empty()) {
        data_->robot_arm_part_name_ = part_name;
        LOG(INFO) << "Cached robot arm part name: " << part_name;
      }
    }
  }

  // Use cached part names for publishing
  if (data_->robot_joint_state_topic_enabled_ &&
      data_->robot_arm_part_name_.has_value()) {
    const std::string& part_name = data_->robot_arm_part_name_.value();
    auto it = robot_status.status_map().find(part_name);
    if (it != robot_status.status_map().end()) {
      PublishJointState(part_name, data_->robot_base_frame_id_, it->second,
                        time);
    } else {
      LOG_EVERY_N(ERROR, 100)
          << "Error: Robot arm part [" << part_name << "] not found!";
    }
  }
}

void MoveitSceneBridge::PublishJointState(
    const std::string& part_name, const std::string& frame_id,
    const intrinsic_proto::icon::PartStatus& part_status,
    const rclcpp::Time& time) {
  sensor_msgs::msg::JointState robot_joint_state_ros;
  robot_joint_state_ros.header.stamp = time;
  robot_joint_state_ros.header.frame_id = frame_id;

  for (int i = 0; i < part_status.joint_states_size(); ++i) {
    const auto& joint_state = part_status.joint_states(i);
    std::string joint_name = absl::StrFormat("%s_joint_%d", part_name, i);
    if (!data_->override_joint_names_.empty()) {
      if (std::size_t(part_status.joint_states_size()) ==
          data_->override_joint_names_.size()) {
        joint_name = data_->override_joint_names_[i];
      } else {
        LOG_EVERY_N(ERROR, 100)
            << "Size of override_joint_names is not equal to "
               "size of joints from part ["
            << part_name << "]. Using default joint names!";
      }
    }
    robot_joint_state_ros.name.push_back(joint_name);

    double pos = joint_state.has_position_sensed()
                     ? joint_state.position_sensed()
                     : std::numeric_limits<double>::quiet_NaN();
    double vel = joint_state.has_velocity_sensed()
                     ? joint_state.velocity_sensed()
                     : std::numeric_limits<double>::quiet_NaN();
    double eff = joint_state.has_torque_sensed()
                     ? joint_state.torque_sensed()
                     : std::numeric_limits<double>::quiet_NaN();

    robot_joint_state_ros.position.push_back(pos);
    robot_joint_state_ros.velocity.push_back(vel);
    robot_joint_state_ros.effort.push_back(eff);
  }
  data_->robot_joint_state_pub_->publish(robot_joint_state_ros);
}

///=============================================================================

MoveitSceneBridge::~MoveitSceneBridge() {
  if (data_->viz_thread_ && data_->viz_thread_->joinable()) {
    {
      absl::MutexLock lock(&data_->mutex_);
      data_->shutdown_ = true;
    }
    data_->viz_thread_->join();
  }
  data_->viz_thread_.reset();
}

}  // namespace flowstate_ros_bridge

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(flowstate_ros_bridge::MoveitSceneBridge,
                       flowstate_ros_bridge::BridgeInterface)
