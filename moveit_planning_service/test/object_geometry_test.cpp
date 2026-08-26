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

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "gtest/gtest.h"

namespace moveit_planning_service {
namespace {

TEST(ComputeObjectBoundingBoxTest, ReturnsBoxDimensionsAndPose) {
  moveit_msgs::msg::CollisionObject object;

  // Creating the box
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.1, 0.2, 0.3};

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 1.0;
  box_pose.position.y = 2.0;
  box_pose.position.z = 3.0;
  box_pose.orientation.w = 1.0;

  // Adding it to the collision object
  object.primitives.push_back(box);
  object.primitive_poses.push_back(box_pose);

  const auto bounding_box = computeObjectBoundingBox(object);

  ASSERT_TRUE(bounding_box.has_value());

  EXPECT_DOUBLE_EQ(bounding_box->dimensions.x(), 0.1);
  EXPECT_DOUBLE_EQ(bounding_box->dimensions.y(), 0.2);
  EXPECT_DOUBLE_EQ(bounding_box->dimensions.z(), 0.3);

  EXPECT_DOUBLE_EQ(bounding_box->center_transform.translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(bounding_box->center_transform.translation().y(), 2.0);
  EXPECT_DOUBLE_EQ(bounding_box->center_transform.translation().z(), 3.0);
}

TEST(ComputeObjectBoundingBoxTest, ComputesDimensionsFromMeshVertices) {
  moveit_msgs::msg::CollisionObject object;

  // Creating a mesh with simple vertices
  shape_msgs::msg::Mesh mesh;

  geometry_msgs::msg::Point minimum_vertex;
  minimum_vertex.x = -0.1;
  minimum_vertex.y = -0.2;
  minimum_vertex.z = -0.3;

  geometry_msgs::msg::Point maximum_vertex;
  maximum_vertex.x = 0.1;
  maximum_vertex.y = 0.2;
  maximum_vertex.z = 0.3;

  mesh.vertices.push_back(minimum_vertex);
  mesh.vertices.push_back(maximum_vertex);

  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.position.x = 1.0;
  mesh_pose.position.y = 2.0;
  mesh_pose.position.z = 3.0;
  mesh_pose.orientation.w = 1.0;

  object.meshes.push_back(mesh);
  object.mesh_poses.push_back(mesh_pose);

  const auto bounding_box = computeObjectBoundingBox(object);

  ASSERT_TRUE(bounding_box.has_value());

  EXPECT_NEAR(bounding_box->dimensions.x(), 0.2, 1e-6);
  EXPECT_NEAR(bounding_box->dimensions.y(), 0.4, 1e-6);
  EXPECT_NEAR(bounding_box->dimensions.z(), 0.6, 1e-6);

  EXPECT_NEAR(bounding_box->center_transform.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(bounding_box->center_transform.translation().y(), 2.0, 1e-6);
  EXPECT_NEAR(bounding_box->center_transform.translation().z(), 3.0, 1e-6);
}

TEST(ComputeObjectBoundingBoxTest, ReturnsNulloptForEmptyObject) {
  moveit_msgs::msg::CollisionObject object;

  const auto bounding_box = computeObjectBoundingBox(object);

  EXPECT_FALSE(bounding_box.has_value());
}

TEST(ComputeObjectBoundingBoxTest, ReturnsNulloptForWrongMesh) {
  moveit_msgs::msg::CollisionObject object;
  shape_msgs::msg::Mesh mesh;

  geometry_msgs::msg::Point vertex;
  vertex.x = 0.1;
  vertex.y = 0.2;
  vertex.z = 0.3;

  mesh.vertices.push_back(vertex);
  object.meshes.push_back(mesh);

  const auto bounding_box = computeObjectBoundingBox(object);

  EXPECT_FALSE(bounding_box.has_value());
}

}  // namespace
}  // namespace moveit_planning_service
