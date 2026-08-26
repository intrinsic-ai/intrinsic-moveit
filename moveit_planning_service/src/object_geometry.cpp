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

#include "moveit_planning_service/object_geometry.hpp"

#include <limits>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "moveit_planning_service/generate_box_grasp_poses.hpp"

namespace moveit_planning_service {

std::optional<ObjectBoundingBox> computeObjectBoundingBox(
    const moveit_msgs::msg::CollisionObject& object) {
  // Prefer a BOX primitive because its dimensions and orientation are known
  // directly, without approximating the object with an axis-aligned box.
  for (size_t i = 0; i < object.primitives.size(); ++i) {
    const auto& primitive = object.primitives[i];
    if (primitive.type != shape_msgs::msg::SolidPrimitive::BOX ||
        primitive.dimensions.size() <= shape_msgs::msg::SolidPrimitive::BOX_Z) {
      continue;  // Skip non-box primitives or malformed boxes
    }

    const Eigen::Vector3d dimensions(
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X],
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y],
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z]);

    if ((dimensions.array() <= 0.0).any()) {
      continue;  // Skip boxes with non-positive dimensions
    }

    Eigen::Isometry3d center_transform = Eigen::Isometry3d::Identity();
    if (i < object.primitive_poses
                .size()) {  // Validate that the pose exists for this primitive
      center_transform = pose_to_isometry(object.primitive_poses[i]);
    }
    return ObjectBoundingBox{dimensions, center_transform};
  }

  // If the object has no BOX primitive, compute a bounding box from all mesh
  // vertices.
  Eigen::Vector3d minimum =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

  bool found_vertex = false;

  for (size_t i = 0; i < object.meshes.size(); ++i) {
    Eigen::Isometry3d mesh_transform = Eigen::Isometry3d::Identity();
    if (i < object.mesh_poses
                .size()) {  // Validate that the mesh pose exists for this mesh
      mesh_transform = pose_to_isometry(object.mesh_poses[i]);
    }

    for (const auto& vertex : object.meshes[i].vertices) {
      const Eigen::Vector3d vertex_in_object_frame =
          mesh_transform * Eigen::Vector3d(vertex.x, vertex.y, vertex.z);

      minimum = minimum.cwiseMin(vertex_in_object_frame);
      maximum = maximum.cwiseMax(vertex_in_object_frame);
      found_vertex = true;
    }
  }
  if (!found_vertex) {
    return std::nullopt;  // No vertices found in any mesh
  }

  const Eigen::Vector3d dimensions = maximum - minimum;
  if ((dimensions.array() <= 0.0).any()) {
    return std::nullopt;  // Skip boxes with non-positive dimensions
  }

  Eigen::Isometry3d center_transform = Eigen::Isometry3d::Identity();
  center_transform.translation() = 0.5 * (minimum + maximum);

  return ObjectBoundingBox{dimensions, center_transform};
}
}  // namespace moveit_planning_service
