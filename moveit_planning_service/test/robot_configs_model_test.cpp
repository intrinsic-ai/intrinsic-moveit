// Copyright 2026 Intrinsic Innovation LLC

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "moveit_planning_service/grasp_planning_pipeline.hpp"

namespace {

struct RobotConfigTestCase {
  std::string config_name;
  double expected_tcp_z;
  double expected_elbow_x;
  double expected_wrist_max_vel;
};

std::string RunCommandCaptureStdout(const std::string& cmd, int* exit_code) {
  std::array<char, 4096> buffer{};
  std::string output;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    *exit_code = -1;
    return output;
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) !=
         nullptr) {
    output.append(buffer.data());
  }
  *exit_code = pclose(pipe);
  return output;
}

std::string ReadFileToString(const std::filesystem::path& path) {
  std::ifstream ifs(path);
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

std::string FormatContacts(
    const collision_detection::CollisionResult::ContactMap& contacts) {
  std::ostringstream oss;
  for (const auto& [pair, contact_vec] : contacts) {
    oss << "[" << pair.first << " <-> " << pair.second << " ("
        << contact_vec.size() << ")] ";
  }
  return oss.str();
}

class RobotConfigsModelTest
    : public ::testing::TestWithParam<RobotConfigTestCase> {
 protected:
  moveit::core::RobotModelPtr LoadRobotModel(const RobotConfigTestCase& tc) {
    const std::string pkg_name = tc.config_name + "_moveit_config";
    std::string share_dir_str;
    try {
      share_dir_str = ament_index_cpp::get_package_share_directory(pkg_name);
    } catch (const std::exception& e) {
      ADD_FAILURE() << "Package '" << pkg_name << "' not found: " << e.what();
      return nullptr;
    }

    const std::filesystem::path share_dir(share_dir_str);
    const auto xacro_path =
        share_dir / "config" / (tc.config_name + ".urdf.xacro");
    const auto srdf_path = share_dir / "config" / (tc.config_name + ".srdf");
    if (!std::filesystem::is_regular_file(xacro_path)) {
      ADD_FAILURE() << "Missing xacro file: " << xacro_path;
      return nullptr;
    }
    if (!std::filesystem::is_regular_file(srdf_path)) {
      ADD_FAILURE() << "Missing srdf file: " << srdf_path;
      return nullptr;
    }

    int exit_code = 0;
    const std::string urdf_xml =
        RunCommandCaptureStdout("xacro " + xacro_path.string(), &exit_code);
    if (exit_code != 0 || urdf_xml.empty()) {
      ADD_FAILURE() << "xacro failed for " << xacro_path
                    << " (exit_code=" << exit_code << ")";
      return nullptr;
    }

    const auto urdf_model = urdf::parseURDF(urdf_xml);
    if (!urdf_model) {
      ADD_FAILURE() << "Failed to parse URDF XML for " << tc.config_name;
      return nullptr;
    }

    const std::string srdf_xml = ReadFileToString(srdf_path);
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, srdf_xml)) {
      ADD_FAILURE() << "Failed to parse SRDF XML for " << tc.config_name;
      return nullptr;
    }

    return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  }
};

TEST_P(RobotConfigsModelTest, SemanticKinematicsAndCollisionInvariantsHold) {
  const auto& tc = GetParam();
  const auto robot_model = LoadRobotModel(tc);
  ASSERT_NE(robot_model, nullptr);

  EXPECT_EQ(robot_model->getName(), tc.config_name);
  EXPECT_EQ(robot_model->getModelFrame(), "base_link");
  ASSERT_NE(robot_model->getSRDF(), nullptr);
  const auto& virtual_joints = robot_model->getSRDF()->getVirtualJoints();
  ASSERT_EQ(virtual_joints.size(), 1u);
  EXPECT_EQ(virtual_joints[0].name_, "virtual_joint");
  EXPECT_EQ(virtual_joints[0].type_, "fixed");
  EXPECT_EQ(virtual_joints[0].parent_frame_, "world");
  EXPECT_EQ(virtual_joints[0].child_link_, "base_link");

  // Required links across all workcell configurations.
  const std::vector<std::string> required_links = {
      "base_link",
      "flange",
      "tool0",
      "hande_hande_base_link",
      "gripper_base_link",
      "hande_tcp",
      "hande_tool_frame",
      "tool_frame",
      "hande_hande_finger_link_l",
      "hande_hande_finger_link_r",
      "orbbec_camera_link",
      "orbbec_camera_mesh_centroid_frame",
  };
  for (const auto& link_name : required_links) {
    EXPECT_NE(robot_model->getLinkModel(link_name), nullptr)
        << "Missing link '" << link_name << "' in " << tc.config_name;
  }

  // Manipulator group invariants.
  const auto* arm_jmg = robot_model->getJointModelGroup("ur_manipulator");
  ASSERT_NE(arm_jmg, nullptr);
  const std::vector<std::string> expected_arm_joints = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
  };
  EXPECT_EQ(arm_jmg->getActiveJointModelNames(), expected_arm_joints);

  std::map<std::string, double> state_values;
  EXPECT_TRUE(arm_jmg->getVariableDefaultPositions("home", state_values));
  EXPECT_TRUE(arm_jmg->getVariableDefaultPositions("ready", state_values));

  // End-effector group invariants.
  const auto* hand_jmg = robot_model->getJointModelGroup("hand");
  ASSERT_NE(hand_jmg, nullptr);
  ASSERT_NE(robot_model->getEndEffector("hand"), nullptr);
  EXPECT_EQ(hand_jmg->getEndEffectorParentGroup().first, "ur_manipulator");
  EXPECT_EQ(hand_jmg->getEndEffectorParentGroup().second, "flange");

  std::map<std::string, double> open_map;
  std::map<std::string, double> closed_map;
  ASSERT_TRUE(hand_jmg->getVariableDefaultPositions("open", open_map));
  ASSERT_TRUE(hand_jmg->getVariableDefaultPositions("closed", closed_map));
  EXPECT_NEAR(open_map.at("hande_left_finger_joint"), -0.001, 1e-6);
  EXPECT_NEAR(closed_map.at("hande_left_finger_joint"), 0.025, 1e-6);

  // wrist_3_joint bounds (+-2*pi) and velocity limits.
  const auto* wrist3 = robot_model->getJointModel("wrist_3_joint");
  ASSERT_NE(wrist3, nullptr);
  const auto& bounds = wrist3->getVariableBounds();
  ASSERT_EQ(bounds.size(), 1u);
  EXPECT_TRUE(bounds[0].position_bounded_);
  EXPECT_NEAR(bounds[0].min_position_, -2.0 * M_PI, 1e-5);
  EXPECT_NEAR(bounds[0].max_position_, 2.0 * M_PI, 1e-5);
  EXPECT_NEAR(bounds[0].max_velocity_, tc.expected_wrist_max_vel, 1e-5);

  // Kinematic TCP & arm link dimensions in default state.
  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  state.update();

  const Eigen::Isometry3d flange_tf = state.getGlobalLinkTransform("flange");
  const Eigen::Isometry3d tcp_tf = state.getGlobalLinkTransform("hande_tcp");
  const Eigen::Isometry3d prefixed_tool_tf =
      state.getGlobalLinkTransform("hande_tool_frame");
  const Eigen::Isometry3d tool_tf = state.getGlobalLinkTransform("tool_frame");

  const Eigen::Isometry3d flange_to_tcp = flange_tf.inverse() * tcp_tf;
  EXPECT_NEAR(flange_to_tcp.translation().x(), 0.0, 1e-6);
  EXPECT_NEAR(flange_to_tcp.translation().y(), 0.0, 1e-6);
  EXPECT_NEAR(flange_to_tcp.translation().z(), tc.expected_tcp_z, 1e-6);
  EXPECT_TRUE(tcp_tf.isApprox(prefixed_tool_tf, 1e-6));
  EXPECT_TRUE(tcp_tf.isApprox(tool_tf, 1e-6));

  const Eigen::Isometry3d upper_tf =
      state.getGlobalLinkTransform("upper_arm_link");
  const Eigen::Isometry3d forearm_tf =
      state.getGlobalLinkTransform("forearm_link");
  const Eigen::Isometry3d upper_to_forearm = upper_tf.inverse() * forearm_tf;
  EXPECT_NEAR(upper_to_forearm.translation().x(), tc.expected_elbow_x, 1e-5);

  // Self-collision checks across zero, home, and ready states (open + closed).
  planning_scene::PlanningScene scene(robot_model);

  const auto& acm = scene.getAllowedCollisionMatrix();
  collision_detection::AllowedCollision::Type allowed_type;
  EXPECT_FALSE(
      acm.getAllowedCollision("base_link", "wrist_3_link", allowed_type));
  EXPECT_FALSE(acm.getAllowedCollision("base_link", "hande_hande_base_link",
                                       allowed_type));
  EXPECT_FALSE(acm.getAllowedCollision("upper_arm_link",
                                       "hande_hande_base_link", allowed_type));

  for (const std::string& arm_state_name :
       {std::string(""), std::string("home"), std::string("ready")}) {
    for (const std::string& hand_state_name :
         {std::string("open"), std::string("closed")}) {
      moveit::core::RobotState test_state(robot_model);
      test_state.setToDefaultValues();
      if (!arm_state_name.empty()) {
        ASSERT_TRUE(test_state.setToDefaultValues(arm_jmg, arm_state_name));
      }
      ASSERT_TRUE(test_state.setToDefaultValues(hand_jmg, hand_state_name));
      test_state.update();

      collision_detection::CollisionRequest req;
      req.contacts = true;
      req.max_contacts = 20;
      collision_detection::CollisionResult res;
      scene.checkSelfCollision(req, res, test_state);
      EXPECT_FALSE(res.collision)
          << "Unexpected self-collision in config='" << tc.config_name
          << "' arm='" << (arm_state_name.empty() ? "zeros" : arm_state_name)
          << "' hand='" << hand_state_name
          << "': " << FormatContacts(res.contacts);
    }
  }

  // Positive control: folding elbow + wrist back into upper_arm_link must be
  // detected as a real self-collision.
  moveit::core::RobotState colliding_state(robot_model);
  colliding_state.setToDefaultValues();
  colliding_state.setJointPositions("elbow_joint", std::vector<double>{2.75});
  colliding_state.setJointPositions("wrist_1_joint", std::vector<double>{1.8});
  colliding_state.update();
  collision_detection::CollisionRequest pos_req;
  collision_detection::CollisionResult pos_res;
  scene.checkSelfCollision(pos_req, pos_res, colliding_state);
  EXPECT_TRUE(pos_res.collision)
      << "Self-collision check failed to detect folded-arm collision in "
      << tc.config_name;
}

TEST_P(RobotConfigsModelTest, GraspPlanningPipelineAcceptsRobotModel) {
  const auto& tc = GetParam();
  const auto robot_model = LoadRobotModel(tc);
  ASSERT_NE(robot_model, nullptr);

  auto node = std::make_shared<rclcpp::Node>("robot_configs_pipeline_test_" +
                                             tc.config_name);
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/unused_planning_scene_" + tc.config_name);

  moveit_planning_service::GraspPlanningPipeline pipeline(node, client,
                                                          robot_model);
  EXPECT_EQ(pipeline.GetRobotModel(), robot_model);

  // Without explicit dimensions and no running GetPlanningScene service,
  // PlanGrasps returns INVALID_OBJECT_NAME cleanly; with explicit dimensions
  // and a bogus end-effector name it returns INVALID_ROBOT_STATE, whereas with
  // 'hand' + 'hande_tcp' it passes semantic validation on the RobotModel.
  moveit_planning_interfaces::action::PlanGrasps::Goal goal;
  goal.group_name = "ur_manipulator";
  goal.end_effector_group = "nonexistent_eef";
  goal.tool_frame = "hande_tcp";

  moveit_planning_interfaces::msg::GraspCandidate candidate;
  candidate.target.id = "box";
  candidate.obj_dims_in_meters.x = 0.04;
  candidate.obj_dims_in_meters.y = 0.04;
  candidate.obj_dims_in_meters.z = 0.04;

  moveit_planning_interfaces::action::PlanGrasps::Result bad_eef_res;
  pipeline.PlanGrasps(goal, candidate, 0, bad_eef_res,
                      std::chrono::milliseconds(20));
  EXPECT_EQ(bad_eef_res.error_code.val,
            moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);

  goal.end_effector_group = "hand";
  moveit_planning_interfaces::action::PlanGrasps::Result valid_model_res;
  pipeline.PlanGrasps(goal, candidate, 0, valid_model_res,
                      std::chrono::milliseconds(20));
  EXPECT_NE(valid_model_res.error_code.val,
            moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
  EXPECT_NE(valid_model_res.error_code.val,
            moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME);
}

INSTANTIATE_TEST_SUITE_P(
    MultiCellConfigs, RobotConfigsModelTest,
    ::testing::Values(RobotConfigTestCase{"omts", 0.14, 0.425, 3.1415927},
                      RobotConfigTestCase{"lab_bb_01", 0.128, 0.24355,
                                          6.2831853}),
    [](const ::testing::TestParamInfo<RobotConfigTestCase>& info) {
      return info.param.config_name;
    });

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  int rc = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
