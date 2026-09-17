// Copyright 2026 Intrinsic Innovation LLC

#include "moveit_planning_service/moveit_scene_synchronizer.hpp"

#include <geometric_shapes/mesh_operations.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/strip.h"
#include "absl/synchronization/mutex.h"
#include "geometry_msgs/msg/pose.hpp"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/eigen.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace moveit_planning_service {

struct MoveitSceneSynchronizer::Data
    : public std::enable_shared_from_this<MoveitSceneSynchronizer::Data> {
  ~Data() = default;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<flowstate_ros_bridge::World> world_;

  std::shared_ptr<intrinsic::Subscription> tf_sub_;

  // MoveIt Collision Object publisher
  std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::CollisionObject>>
      collision_objects_pub_;
  std::string tf_prefix_;
  std::vector<std::string> strip_tf_prefixes_;
  std::vector<std::string> excluded_collision_namespaces_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>>
      sync_collision_objects_srv_;

  // Maps mesh_id to glTF binary data for a collision mesh
  absl::flat_hash_map<std::string, std::vector<uint8_t>> renderables_collision_
      ABSL_GUARDED_BY(mutex_);
  // Maps mesh_id to primitive shape data
  absl::flat_hash_map<std::string, shape_msgs::msg::SolidPrimitive>
      primitives_collision_ ABSL_GUARDED_BY(mutex_);
  // Maps the TF frame of an object to multiple mesh_ids
  absl::flat_hash_map<std::string, absl::flat_hash_set<std::string>>
      collision_tf_frame_to_mesh_ids_ ABSL_GUARDED_BY(mutex_);
  // Maps a mesh_id to the pose of the mesh relative to its TF frame
  absl::flat_hash_map<std::string, geometry_msgs::msg::Pose> relative_mesh_pose_
      ABSL_GUARDED_BY(mutex_);
  absl::flat_hash_set<std::string> tf_frame_names_;

  std::optional<std::vector<std::string>> sync_object_names_
      ABSL_GUARDED_BY(mutex_) = std::nullopt;
  bool sync_new_objects_ ABSL_GUARDED_BY(mutex_) = true;
  bool shutdown_ ABSL_GUARDED_BY(mutex_) = false;
  std::shared_ptr<std::thread> sync_thread_;
  mutable absl::Mutex mutex_;

  // Throttling & deadbanding for collision object pose updates
  double collision_object_update_rate_hz_{10.0};
  double collision_object_min_position_delta_{0.001};  // 1 mm
  double collision_object_min_rotation_delta_{0.01};   // ~0.57 degrees
  double world_sync_interval_sec_{2.0};  // Periodic background sync
  absl::flat_hash_map<std::string, geometry_msgs::msg::Pose>
      last_published_collision_pose_ ABSL_GUARDED_BY(mutex_);
  absl::flat_hash_map<std::string, rclcpp::Time> last_published_collision_time_
      ABSL_GUARDED_BY(mutex_);

  absl::Status FetchAndSynchronizeCollisionObjects(
      std::optional<std::vector<std::string>> object_names);
  void addCollisionObjectsToScene(
      std::optional<std::vector<std::string>> object_frames);
  void updateCollisionObjects(
      const std::vector<geometry_msgs::msg::TransformStamped>& transforms);
  void removeCollisionObject(const std::string& tf_frame_name);
};

///=============================================================================
MoveitSceneSynchronizer::MoveitSceneSynchronizer()
    : data_(std::make_shared<Data>()) {}

MoveitSceneSynchronizer::~MoveitSceneSynchronizer() {
  if (data_) {
    if (data_->sync_thread_ && data_->sync_thread_->joinable()) {
      {
        absl::MutexLock lock(&data_->mutex_);
        data_->shutdown_ = true;
      }
      data_->sync_thread_->join();
    }
    data_->sync_thread_.reset();
  }
}

std::size_t MoveitSceneSynchronizer::getTrackedCollisionObjectsCount() const {
  if (!data_) return 0;
  absl::MutexLock lock(&data_->mutex_);
  return data_->collision_tf_frame_to_mesh_ids_.size();
}

std::string MoveitSceneSynchronizer::StripTfPrefixes(
    absl::string_view frame, const std::vector<std::string>& prefixes) {
  absl::string_view stripped = frame;
  for (const auto& prefix : prefixes) {
    if (!prefix.empty() && absl::StartsWith(stripped, prefix)) {
      stripped = absl::StripPrefix(stripped, prefix);
      break;
    }
  }
  return std::string(stripped);
}

///=============================================================================
bool MoveitSceneSynchronizer::initialize(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<flowstate_ros_bridge::World>& world_client,
    const MoveitSceneSynchronizerConfig& config) {
  if (!node || !world_client) {
    LOG(ERROR) << "MoveitSceneSynchronizer::initialize called with null node "
                  "or world client.";
    return false;
  }

  data_->node_ = node;
  data_->world_ = world_client;

  // Unpack configuration parameters from typed struct
  data_->tf_prefix_ = config.tf_prefix;
  data_->strip_tf_prefixes_ = config.strip_tf_prefixes;
  data_->excluded_collision_namespaces_ = config.excluded_collision_namespaces;
  data_->collision_object_update_rate_hz_ =
      config.collision_objects_update_rate_hz;
  data_->collision_object_min_position_delta_ =
      config.collision_objects_min_position_delta;
  data_->collision_object_min_rotation_delta_ =
      config.collision_objects_min_rotation_delta;
  data_->world_sync_interval_sec_ = config.world_sync_interval_sec;

  std::weak_ptr<Data> data_wp = data_;

  // Trigger service for collision object synchronization
  data_->sync_collision_objects_srv_ = node->create_service<
      std_srvs::srv::Trigger>(
      "~/sync_collision_objects",
      [data_wp](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto data = data_wp.lock();
        if (!data) {
          response->success = false;
          response->message = "MoveitSceneSynchronizer data deallocated";
          return;
        }
        const auto status =
            data->FetchAndSynchronizeCollisionObjects(std::nullopt);
        data->addCollisionObjectsToScene(std::nullopt);
        if (status.ok()) {
          response->success = true;
          response->message = "Collision objects synchronized to MoveIt scene";
        } else {
          response->success = false;
          response->message =
              std::string("Failed to fetch collision objects: ") +
              status.ToString();
        }
      },
      rclcpp::ServicesQoS());

  constexpr const char* kCollisionTopic = "/collision_object";

  // Publisher for MoveIt collision objects
  data_->collision_objects_pub_ =
      node->create_publisher<moveit_msgs::msg::CollisionObject>(
          kCollisionTopic, rclcpp::ServicesQoS());

  // Subscribe to World TF for dynamic collision object tracking
  auto tf_sub = data_->world_->CreateTfSubscription(
      [data_wp](const intrinsic_proto::TFMessage& msg) {
        auto data = data_wp.lock();
        if (!data || !data->node_) {
          return;
        }
        const rclcpp::Time t_start = data->node_->now();

        absl::flat_hash_set<std::string> new_tf_frame_names;
        absl::flat_hash_set<std::string> new_object_names;

        tf2_msgs::msg::TFMessage tf_ros;
        tf_ros.transforms = std::vector<geometry_msgs::msg::TransformStamped>(
            msg.transforms_size());
        int tf_idx = 0;
        for (const auto& ts_proto : msg.transforms()) {
          geometry_msgs::msg::TransformStamped* ts_ros =
              &tf_ros.transforms[tf_idx++];
          ts_ros->header.stamp = t_start;

          const std::string frame_id = StripTfPrefixes(
              ts_proto.header().frame_id(), data->strip_tf_prefixes_);
          const std::string child_frame_id = StripTfPrefixes(
              ts_proto.child_frame_id(), data->strip_tf_prefixes_);

          ts_ros->header.frame_id = data->tf_prefix_ + frame_id;
          ts_ros->child_frame_id = data->tf_prefix_ + child_frame_id;

          new_tf_frame_names.insert(ts_ros->child_frame_id);
          if (!data->tf_frame_names_.contains(ts_ros->child_frame_id)) {
            LOG(INFO) << "Discovered new TF child_frame_id: "
                      << ts_ros->child_frame_id;
            const std::size_t str_end = child_frame_id.find('/');
            new_object_names.insert(child_frame_id.substr(0, str_end));
          }

          const auto& t = ts_proto.transform();
          ts_ros->transform.translation.x = t.translation().x();
          ts_ros->transform.translation.y = t.translation().y();
          ts_ros->transform.translation.z = t.translation().z();
          ts_ros->transform.rotation.x = t.rotation().x();
          ts_ros->transform.rotation.y = t.rotation().y();
          ts_ros->transform.rotation.z = t.rotation().z();
          ts_ros->transform.rotation.w = t.rotation().w();
        }

        // Remove deleted collision frames
        for (const auto& tf_frame : data->tf_frame_names_) {
          if (!new_tf_frame_names.contains(tf_frame)) {
            data->removeCollisionObject(tf_frame);
            LOG(INFO) << "Removed collision object with frame_id " << tf_frame;
          }
        }

        if (!new_object_names.empty()) {
          absl::MutexLock lock(&data->mutex_);
          if (data->sync_object_names_.has_value()) {
            data->sync_object_names_.value().insert(
                data->sync_object_names_.value().end(),
                new_object_names.begin(), new_object_names.end());
          } else {
            data->sync_object_names_ = std::vector<std::string>(
                new_object_names.begin(), new_object_names.end());
          }
          data->sync_new_objects_ = true;
        }

        data->tf_frame_names_ = std::move(new_tf_frame_names);
        data->updateCollisionObjects(tf_ros.transforms);
      });

  if (!tf_sub.ok()) {
    LOG(ERROR) << "Unable to create World TF Subscription: " << tf_sub.status();
    return false;
  }
  LOG(INFO) << "Subscribed to World TF topic for tracking the transform of "
               "collision objects";
  data_->tf_sub_ = std::move(*tf_sub);

  // Background thread for asynchronous collision object retrieval &
  // synchronization
  data_->sync_thread_ = std::make_shared<std::thread>([data_wp]() {
    while (rclcpp::ok()) {
      if (auto data = data_wp.lock()) {
        std::optional<std::vector<std::string>> sync_object_names;
        {
          absl::MutexLock lock(&data->mutex_);
          if (data->world_sync_interval_sec_ > 0.0) {
            data->mutex_.AwaitWithTimeout(
                absl::Condition(
                    +[](Data* d) {
                      return d->sync_new_objects_ || d->shutdown_;
                    },
                    data.get()),
                absl::Milliseconds(static_cast<int64_t>(
                    data->world_sync_interval_sec_ * 1000.0)));
          } else {
            data->mutex_.Await(absl::Condition(
                +[](Data* d) { return d->sync_new_objects_ || d->shutdown_; },
                data.get()));
          }

          if (data->shutdown_) {
            break;
          }
          sync_object_names = std::move(data->sync_object_names_);
          data->sync_object_names_ = std::nullopt;
          data->sync_new_objects_ = false;
        }

        const absl::Status status =
            data->FetchAndSynchronizeCollisionObjects(sync_object_names);
        if (!status.ok()) {
          LOG(ERROR) << "Unable to synchronize collision objects: "
                     << status.message();
          continue;
        }
        data->addCollisionObjectsToScene(sync_object_names);
      } else {
        return;
      }
    }
  });

  return true;
}

///=============================================================================
absl::Status MoveitSceneSynchronizer::fetchAndSynchronizeCollisionObjects(
    std::optional<std::vector<std::string>> object_frames) {
  if (!data_ || !data_->world_) {
    return absl::FailedPreconditionError(
        "MoveitSceneSynchronizer is not initialized with a valid World client");
  }
  const absl::Status status =
      data_->FetchAndSynchronizeCollisionObjects(object_frames);
  if (!status.ok()) {
    return status;
  }
  data_->addCollisionObjectsToScene(object_frames);
  return absl::OkStatus();
}

void MoveitSceneSynchronizer::addCollisionObjectsToScene(
    std::optional<std::vector<std::string>> object_frames) {
  if (data_ && data_->node_ && data_->collision_objects_pub_) {
    data_->addCollisionObjectsToScene(std::move(object_frames));
  }
}

void MoveitSceneSynchronizer::removeCollisionObject(
    const std::string& tf_frame_name) {
  if (data_ && data_->node_ && data_->collision_objects_pub_) {
    data_->removeCollisionObject(tf_frame_name);
  }
}

void MoveitSceneSynchronizer::updateCollisionObjects(
    const std::vector<geometry_msgs::msg::TransformStamped>& transforms) {
  if (data_ && data_->node_ && data_->collision_objects_pub_) {
    data_->updateCollisionObjects(transforms);
  }
}

///=============================================================================
absl::Status MoveitSceneSynchronizer::Data::FetchAndSynchronizeCollisionObjects(
    std::optional<std::vector<std::string>> object_names) {
  if (!world_) {
    return absl::FailedPreconditionError("World client is not initialized");
  }
  if (object_names.has_value() && object_names->empty()) {
    object_names = std::nullopt;
  }
  absl::StatusOr<std::vector<intrinsic::world::WorldObject>> objects =
      world_->GetObjects(std::move(object_names));
  if (!objects.ok()) {
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "GetObjects failed: %s",
                   objects.status().ToString().c_str());
    }
    return objects.status();
  }

  for (const intrinsic::world::WorldObject& object : *objects) {
    if (object.Name().empty()) {
      continue;
    }
    const intrinsic_proto::world::Object& proto = object.Proto();
    for (const auto& entity : proto.entities()) {
      if (!entity.second.has_geometry_component()) {
        continue;
      }
      for (const auto& named_geometry :
           entity.second.geometry_component().named_geometries()) {
        if (named_geometry.first != "Intrinsic_Collision") {
          continue;
        }

        const auto& named_geoms = named_geometry.second.named_geometries();

        for (const auto& [inner_name, transformed_geometry] : named_geoms) {
          const std::string object_name =
              StripTfPrefixes(object.Name().value(), strip_tf_prefixes_);
          const std::vector<absl::string_view> parts =
              absl::StrSplit(object_name, '/');
          const absl::string_view short_name = parts.back();

          bool is_excluded = false;
          for (const auto& ns : excluded_collision_namespaces_) {
            if (short_name == ns) {
              is_excluded = true;
              break;
            }
          }
          if (is_excluded) {
            continue;
          }

          const std::string tf_frame_name = absl::StrFormat(
              "%s%s/%s", tf_prefix_.c_str(), short_name, entity.second.name());
          const std::string collision_mesh_id =
              std::string("/") + tf_frame_name + std::string("/") +
              std::string(inner_name);

          // Calculate relative pose from ref_t_shape
          geometry_msgs::msg::Pose mesh_pose;
          mesh_pose.orientation.w = 1.0;
          const auto& ref_t_shape = transformed_geometry.ref_t_shape();
          if (ref_t_shape.has_matrix4d()) {
            const absl::StatusOr<intrinsic::eigenmath::MatrixXd> transform_xd =
                intrinsic_proto::FromProto(ref_t_shape.matrix4d());
            if (transform_xd.ok() && transform_xd->rows() == 4 &&
                transform_xd->cols() == 4) {
              const intrinsic::eigenmath::Matrix4d mat4 = *transform_xd;
              const intrinsic::eigenmath::AffineTransform3d affine(mat4);
              mesh_pose.position.x = affine.translation().x();
              mesh_pose.position.y = affine.translation().y();
              mesh_pose.position.z = affine.translation().z();
              const intrinsic::eigenmath::Quaterniond quat(affine.rotation());
              mesh_pose.orientation.x = quat.x();
              mesh_pose.orientation.y = quat.y();
              mesh_pose.orientation.z = quat.z();
              mesh_pose.orientation.w = quat.w();
            }
          }

          const auto& geometry = transformed_geometry.geometry();
          if (geometry.has_geo_ref()) {
            const auto& geo_ref = geometry.geo_ref();
            bool should_fetch = false;
            {
              absl::MutexLock lock(&mutex_);
              should_fetch =
                  !renderables_collision_.contains(collision_mesh_id);
            }

            if (should_fetch) {
              const absl::StatusOr<std::string> gltf = world_->GetGltf(
                  geo_ref.exact_geometry_ref(), geo_ref.renderable_ref());
              if (!gltf.ok()) {
                LOG(ERROR) << "Unable to fetch collision mesh for "
                           << tf_frame_name << ": " << gltf.status();
                continue;
              }

              std::vector<uint8_t> gltf_data(gltf->begin(), gltf->end());

              LOG(INFO) << "Fetched collision mesh (" << gltf_data.size()
                        << " bytes) for " << tf_frame_name;

              absl::MutexLock lock(&mutex_);
              renderables_collision_.emplace(collision_mesh_id,
                                             std::move(gltf_data));
            }

            {
              absl::MutexLock lock(&mutex_);
              relative_mesh_pose_[collision_mesh_id] = mesh_pose;
              collision_tf_frame_to_mesh_ids_[tf_frame_name].insert(
                  collision_mesh_id);
            }
          } else if (geometry.has_inline_geometry_data()) {
            const auto& inline_geometry = geometry.inline_geometry_data();
            if (inline_geometry.has_exact_geometry()) {
              const auto& exact_geometry = inline_geometry.exact_geometry();
              if (exact_geometry.has_primitive_set()) {
                const auto& primitive_set = exact_geometry.primitive_set();
                for (int i = 0; i < primitive_set.primitives_size(); ++i) {
                  const auto& transformed_primitive =
                      primitive_set.primitives(i);
                  if (transformed_primitive.has_shape()) {
                    const auto& primitive_shape = transformed_primitive.shape();

                    shape_msgs::msg::SolidPrimitive solid_prim;
                    if (primitive_shape.has_box()) {
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
                      solid_prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
                      solid_prim.dimensions.resize(1);
                      solid_prim.dimensions
                          [shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] =
                          primitive_shape.sphere().radius();
                    } else if (primitive_shape.has_capsule()) {
                      solid_prim.type =
                          shape_msgs::msg::SolidPrimitive::CYLINDER;
                      solid_prim.dimensions.resize(2);
                      solid_prim.dimensions
                          [shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
                          primitive_shape.capsule().length();
                      solid_prim.dimensions
                          [shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
                          primitive_shape.capsule().radius();
                    } else {
                      LOG(WARNING)
                          << "Unsupported primitive shape for collision: "
                          << tf_frame_name;
                      continue;
                    }

                    const std::string prim_id =
                        primitive_set.primitives_size() > 1
                            ? absl::StrFormat("%s_prim_%d", collision_mesh_id,
                                              i)
                            : collision_mesh_id;

                    {
                      absl::MutexLock lock(&mutex_);
                      primitives_collision_[prim_id] = solid_prim;
                      relative_mesh_pose_[prim_id] = mesh_pose;
                      collision_tf_frame_to_mesh_ids_[tf_frame_name].insert(
                          prim_id);
                    }
                  }
                }
              }
            }
          } else {
            LOG(ERROR) << "No geometry data found for " << tf_frame_name;
            continue;
          }
        }
      }
    }
  }

  return absl::OkStatus();
}

///=============================================================================
void MoveitSceneSynchronizer::Data::addCollisionObjectsToScene(
    std::optional<std::vector<std::string>> object_frames) {
  if (!node_ || !collision_objects_pub_) {
    return;
  }
  const rclcpp::Time t_now = node_->now();

  struct ObjectCollisionData {
    std::string tf_frame_name;
    std::vector<std::pair<std::vector<uint8_t>, geometry_msgs::msg::Pose>>
        gltf_meshes;
    std::vector<
        std::pair<shape_msgs::msg::SolidPrimitive, geometry_msgs::msg::Pose>>
        primitives;
  };

  std::vector<ObjectCollisionData> objects_to_process;
  {
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
      ObjectCollisionData obj_data;
      obj_data.tf_frame_name = tf_frame_name;

      for (const auto& mesh_id : mesh_ids) {
        auto it_pose = relative_mesh_pose_.find(mesh_id);
        if (it_pose == relative_mesh_pose_.end()) {
          continue;
        }
        const auto& relative_pose = it_pose->second;

        if (auto it_mesh = renderables_collision_.find(mesh_id);
            it_mesh != renderables_collision_.end()) {
          obj_data.gltf_meshes.emplace_back(it_mesh->second, relative_pose);
        } else if (auto it_prim = primitives_collision_.find(mesh_id);
                   it_prim != primitives_collision_.end()) {
          obj_data.primitives.emplace_back(it_prim->second, relative_pose);
        }
      }

      objects_to_process.push_back(std::move(obj_data));
    }
  }

  // Decompress glTF meshes and construct MoveIt messages OUTSIDE mutex lock
  std::vector<moveit_msgs::msg::CollisionObject> objects_to_publish;
  for (const auto& obj_data : objects_to_process) {
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.header.stamp = t_now;
    collision_obj.header.frame_id = obj_data.tf_frame_name;
    collision_obj.id = obj_data.tf_frame_name;
    collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_obj.pose.orientation.w = 1.0;

    for (const auto& [gltf_data, relative_pose] : obj_data.gltf_meshes) {
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
        LOG(ERROR) << "Failed to convert glTF mesh for "
                   << obj_data.tf_frame_name;
      }
    }

    for (const auto& [solid_prim, relative_pose] : obj_data.primitives) {
      collision_obj.primitives.push_back(solid_prim);
      collision_obj.primitive_poses.push_back(relative_pose);
    }

    // LOG(INFO) << "Prepared collision object for MoveIt: '"
    //           << obj_data.tf_frame_name << "'";
    objects_to_publish.push_back(std::move(collision_obj));
  }

  for (const auto& collision_obj : objects_to_publish) {
    if (node_) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Published CollisionObject::ADD for '%s' (%zu meshes, %zu "
                   "primitives)",
                   collision_obj.id.c_str(), collision_obj.meshes.size(),
                   collision_obj.primitives.size());
    }
    collision_objects_pub_->publish(collision_obj);
  }
}

///=============================================================================
void MoveitSceneSynchronizer::Data::removeCollisionObject(
    const std::string& tf_frame_name) {
  if (!node_ || !collision_objects_pub_) {
    return;
  }
  const rclcpp::Time t_now = node_->now();
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
  LOG(INFO) << "Removed collision object: '" << tf_frame_name << "'";
}

///=============================================================================
void MoveitSceneSynchronizer::Data::updateCollisionObjects(
    const std::vector<geometry_msgs::msg::TransformStamped>& transforms) {
  if (!node_ || !collision_objects_pub_) {
    return;
  }
  const rclcpp::Time t_now = node_->now();
  std::vector<moveit_msgs::msg::CollisionObject> objects_to_publish;
  {
    absl::MutexLock lock(&mutex_);

    for (const auto& [tf_frame_name, mesh_ids] :
         collision_tf_frame_to_mesh_ids_) {
      for (const auto& ts_ros : transforms) {
        if (ts_ros.child_frame_id == tf_frame_name) {
          geometry_msgs::msg::Pose new_pose;
          new_pose.position.x = ts_ros.transform.translation.x;
          new_pose.position.y = ts_ros.transform.translation.y;
          new_pose.position.z = ts_ros.transform.translation.z;
          new_pose.orientation = ts_ros.transform.rotation;

          if (auto it_time = last_published_collision_time_.find(tf_frame_name);
              it_time != last_published_collision_time_.end()) {
            if (collision_object_update_rate_hz_ > 0.0) {
              const double min_period = 1.0 / collision_object_update_rate_hz_;
              if ((t_now - it_time->second).seconds() < min_period) {
                break;
              }
            }
          }

          if (auto it_pose = last_published_collision_pose_.find(tf_frame_name);
              it_pose != last_published_collision_pose_.end()) {
            const auto& old_pose = it_pose->second;
            const double dx = new_pose.position.x - old_pose.position.x;
            const double dy = new_pose.position.y - old_pose.position.y;
            const double dz = new_pose.position.z - old_pose.position.z;
            const double pos_delta = std::sqrt(dx * dx + dy * dy + dz * dz);

            const auto& q1 = old_pose.orientation;
            const auto& q2 = new_pose.orientation;
            const double dot =
                std::min(1.0, std::abs(q1.w * q2.w + q1.x * q2.x + q1.y * q2.y +
                                       q1.z * q2.z));
            const double rot_delta = 2.0 * std::acos(dot);

            if (pos_delta < collision_object_min_position_delta_ &&
                rot_delta < collision_object_min_rotation_delta_) {
              break;
            }
          }

          moveit_msgs::msg::CollisionObject collision_obj;
          collision_obj.operation = moveit_msgs::msg::CollisionObject::MOVE;
          collision_obj.header.stamp = t_now;
          collision_obj.header.frame_id = ts_ros.header.frame_id;
          collision_obj.id = tf_frame_name;
          collision_obj.pose = new_pose;

          for (const auto& mesh_id : mesh_ids) {
            auto it_pose = relative_mesh_pose_.find(mesh_id);
            if (it_pose == relative_mesh_pose_.end()) {
              continue;
            }
            const auto& relative_pose = it_pose->second;
            if (renderables_collision_.contains(mesh_id)) {
              collision_obj.mesh_poses.push_back(relative_pose);
            } else if (primitives_collision_.contains(mesh_id)) {
              collision_obj.primitive_poses.push_back(relative_pose);
            }
          }

          if (node_) {
            RCLCPP_DEBUG(node_->get_logger(),
                         "Publishing CollisionObject::MOVE for '%s' in frame "
                         "'%s' -> Pos: [%.4f, %.4f, %.4f]",
                         tf_frame_name.c_str(), ts_ros.header.frame_id.c_str(),
                         new_pose.position.x, new_pose.position.y,
                         new_pose.position.z);
          }

          objects_to_publish.push_back(collision_obj);
          last_published_collision_pose_[tf_frame_name] = new_pose;
          last_published_collision_time_[tf_frame_name] = t_now;
          break;
        }
      }
    }
  }

  for (const auto& collision_obj : objects_to_publish) {
    collision_objects_pub_->publish(collision_obj);
  }
}

}  // namespace moveit_planning_service
