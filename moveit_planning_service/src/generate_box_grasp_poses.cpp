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

#include "moveit_planning_service/generate_box_grasp_poses.hpp"

#include <cmath>

namespace moveit_planning_service {

namespace mtc = moveit::task_constructor;

geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry3d& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond q(transform.linear());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  if (q.squaredNorm() > 1e-6) {
    transform.linear() = q.normalized().toRotationMatrix();
  }
  return transform;
}

GenerateBoxGraspPoses::GenerateBoxGraspPoses(const std::string& name)
    : mtc::stages::GeneratePose(name) {
  auto& p = properties();
  p.declare<std::string>("eef", "name of end-effector");
  p.declare<std::string>("object", "target collision object name");
  p.declare<boost::any>("pregrasp", "pregrasp posture");
  p.declare<boost::any>("grasp", "grasp posture");
}

void GenerateBoxGraspPoses::setObject(const std::string& object) {
  setProperty("object", object);
}

void GenerateBoxGraspPoses::setEndEffector(const std::string& eef) {
  setProperty("eef", eef);
}

void GenerateBoxGraspPoses::setPreGraspPose(const std::string& pregrasp) {
  properties().set("pregrasp", pregrasp);
}

void GenerateBoxGraspPoses::setGraspPose(const std::string& grasp) {
  properties().set("grasp", grasp);
}

void GenerateBoxGraspPoses::setEndEffectorPose(const std::string& pose) {
  setPreGraspPose(pose);
  setGraspPose(pose);
}

void GenerateBoxGraspPoses::setSurfaces(const std::vector<int32_t>& surfaces) {
  surfaces_ = surfaces;
}

void GenerateBoxGraspPoses::setNumRotations(int num_rotations) {
  num_rotations_ = num_rotations;
}

void GenerateBoxGraspPoses::setBoxDimensions(const Eigen::Vector3d& dims) {
  dims_ = dims;
}

void GenerateBoxGraspPoses::setCenterTransform(
    const Eigen::Isometry3d& center) {
  obj_t_obj_center_ = center;
}

void GenerateBoxGraspPoses::init(
    const moveit::core::RobotModelConstPtr& robot_model) {
  mtc::InitStageException errors;
  try {
    GeneratePose::init(robot_model);
  } catch (mtc::InitStageException& e) {
    errors.append(e);
  }

  const auto& props = properties();
  props.get<std::string>("object");
  const std::string& eef = props.get<std::string>("eef");
  if (!robot_model->hasEndEffector(eef)) {
    errors.push_back(*this, "unknown end effector: " + eef);
    throw errors;
  }

  if (errors) {
    throw errors;
  }
}

void GenerateBoxGraspPoses::onNewSolution(const mtc::SolutionBase& s) {
  planning_scene::PlanningSceneConstPtr scene = s.end()->scene();
  auto& props = properties();
  std::string object = props.get<std::string>("object");
  if (!scene->knowsFrameTransform(object)) {
    // Flowstate Object World scene objects are exported with the convention "<object_name>/<entity_name>"
    // (e.g. "building_block/whole"). If the root object name was supplied, resolve to the entity frame.
    if (scene->knowsFrameTransform(object + "/whole")) {
      object = object + "/whole";
      props.set("object", object);
    } else {
      const std::string msg = "object '" + object + "' not in scene";
      spawn(mtc::InterfaceState(scene), mtc::SubTrajectory::failure(msg));
      return;
    }
  }
  upstream_solutions_.push(&s);
}

void GenerateBoxGraspPoses::compute() {
  if (upstream_solutions_.empty()) return;

  const mtc::SolutionBase& s = *upstream_solutions_.pop();
  planning_scene::PlanningScenePtr scene = s.end()->scene()->diff();

  const auto& props = properties();
  const std::string& eef = props.get<std::string>("eef");
  const moveit::core::JointModelGroup* jmg =
      scene->getRobotModel()->getEndEffector(eef);

  moveit::core::RobotState& robot_state = scene->getCurrentStateNonConst();
  if (props.hasProperty("pregrasp") && jmg != nullptr) {
    try {
      const auto& pregrasp_val = props.property("pregrasp").value();
      if (!pregrasp_val.empty()) {
        const std::string& pregrasp_name =
            boost::any_cast<std::string>(pregrasp_val);
        robot_state.setToDefaultValues(jmg, pregrasp_name);
      }
    } catch (...) {
    }
  }

  const std::string object_frame = props.get<std::string>("object");
  const int num_rotations = (num_rotations_ > 0) ? num_rotations_ : 4;
  const double angle_step = 2.0 * M_PI / static_cast<double>(num_rotations);

  // Surfaces to process (0: +X, 1: -X, 2: +Y, 3: -Y, 4: +Z, 5: -Z)
  std::vector<int32_t> active_surfaces = surfaces_;
  if (active_surfaces.empty()) {
    active_surfaces = {0, 1, 2, 3, 4, 5};
  }

  for (int32_t surface : active_surfaces) {
    Eigen::Vector3d t_surf = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r_base = Eigen::Matrix3d::Identity();

    // Note: the surface directions are documented as part of
    // https://github.com/intrinsic-ai/sdk/blob/main/intrinsic/manipulation/grasping/grasp_annotations.proto#L88
    switch (surface) {
      case 0:  // +X face
        t_surf = Eigen::Vector3d(dims_.x() / 2.0, 0.0, 0.0);
        r_base = Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY())
                     .toRotationMatrix();
        break;
      case 1:  // -X face
        t_surf = Eigen::Vector3d(-dims_.x() / 2.0, 0.0, 0.0);
        r_base = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY())
                     .toRotationMatrix();
        break;
      case 2:  // +Y face
        t_surf = Eigen::Vector3d(0.0, dims_.y() / 2.0, 0.0);
        r_base = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX())
                     .toRotationMatrix();
        break;
      case 3:  // -Y face
        t_surf = Eigen::Vector3d(0.0, -dims_.y() / 2.0, 0.0);
        r_base = Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX())
                     .toRotationMatrix();
        break;
      case 4:  // +Z face (Top)
        t_surf = Eigen::Vector3d(0.0, 0.0, dims_.z() / 2.0);
        r_base = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                     .toRotationMatrix();
        break;
      case 5:  // -Z face (Bottom)
        t_surf = Eigen::Vector3d(0.0, 0.0, -dims_.z() / 2.0);
        r_base = Eigen::Matrix3d::Identity();
        break;
      default:
        continue;
    }

    for (int i = 0; i < num_rotations; ++i) {
      const double theta = i * angle_step;
      Eigen::Matrix3d r_rot =
          Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

      Eigen::Isometry3d t_surface_local = Eigen::Isometry3d::Identity();
      t_surface_local.translation() = t_surf;
      t_surface_local.linear() = r_base * r_rot;

      Eigen::Isometry3d t_grasp = obj_t_obj_center_ * t_surface_local;

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = object_frame;
      target_pose_msg.pose = isometry_to_pose(t_grasp);

      mtc::InterfaceState state(scene);
      forwardProperties(*s.end(), state);
      state.properties().set("target_pose", target_pose_msg);
      props.exposeTo(state.properties(), {"pregrasp", "grasp"});

      mtc::SubTrajectory trajectory;
      trajectory.setCost(0.0);
      trajectory.setComment("surface_" + std::to_string(surface) + "_rot_" +
                            std::to_string(i));

      spawn(std::move(state), std::move(trajectory));
    }
  }
}

}  // namespace moveit_planning_service
