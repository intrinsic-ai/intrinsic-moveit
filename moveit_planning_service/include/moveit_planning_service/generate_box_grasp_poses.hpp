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

#include <Eigen/Geometry>
#include <boost/any.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/storage.h>
#include <string>
#include <vector>

namespace moveit_planning_service {

/**
 * @brief Helper to convert Eigen::Isometry3d to geometry_msgs::msg::Pose.
 */
geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry3d& transform);

/**
 * @brief Helper to convert geometry_msgs::msg::Pose to Eigen::Isometry3d.
 */
Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose& pose);

/**
 * @brief Generator stage for candidate grasp poses on box surfaces.
 */
class GenerateBoxGraspPoses
    : public moveit::task_constructor::stages::GeneratePose {
 public:
  explicit GenerateBoxGraspPoses(
      const std::string& name = "generate box grasp poses");

  void setObject(const std::string& object);
  void setEndEffector(const std::string& eef);
  void setPreGraspPose(const std::string& pregrasp);
  void setGraspPose(const std::string& grasp);
  void setEndEffectorPose(const std::string& pose);
  void setSurfaces(const std::vector<int32_t>& surfaces);
  void setNumRotations(int num_rotations);
  void setBoxDimensions(const Eigen::Vector3d& dims);
  void setCenterTransform(const Eigen::Isometry3d& center);

  void init(const moveit::core::RobotModelConstPtr& robot_model) override;
  void onNewSolution(const moveit::task_constructor::SolutionBase& s) override;
  void compute() override;

 private:
  std::vector<int32_t> surfaces_;
  int num_rotations_ = 4;
  Eigen::Vector3d dims_{0.05, 0.05, 0.05};
  Eigen::Isometry3d obj_t_obj_center_ = Eigen::Isometry3d::Identity();
};

}  // namespace moveit_planning_service
