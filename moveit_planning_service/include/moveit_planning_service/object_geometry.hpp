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

#pragma once

#include <optional>
#include <Eigen/Geometry>
#include <moveit_msgs/msg/collision_object.hpp>


namespace moveit_planning_service{
    struct ObjectBoundingBox {
        Eigen::Vector3d dimensions;
        Eigen::Isometry3d center_transform;
    };
    /**
     * Extracts a box representation from a collision object.
     *
     * BOX primitives preserve their dimensions and pose. If no BOX primitive is
     * available, a bounding box is calculated from all mesh vertices.
     */
    std::optional<ObjectBoundingBox> computeObjectBoundingBox(const moveit_msgs::msg::CollisionObject& object);
}   // namespace moveit_planning_service
