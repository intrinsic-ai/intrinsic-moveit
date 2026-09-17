// Copyright 2026 Intrinsic Innovation LLC

#include "moveit_planning_service/grasp_planning_pipeline.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_planning_interfaces/srv/plan_grasps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

class GraspPlanningPipelineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("grasp_planning_pipeline_test_node");
    planning_scene_client_ =
        node_->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/test_get_planning_scene");
    pipeline_ =
        std::make_unique<moveit_planning_service::GraspPlanningPipeline>(
            node_, planning_scene_client_);
  }

  void TearDown() override {
    pipeline_.reset();
    planning_scene_client_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  moveit_planning_service::GraspPlanningPipeline::PlanningSceneClient::SharedPtr
      planning_scene_client_;
  std::unique_ptr<moveit_planning_service::GraspPlanningPipeline> pipeline_;
};

TEST_F(GraspPlanningPipelineTest, ResolveTargetObjectIdExactMatch) {
  const std::vector<std::string> scene_ids = {"table", "bin", "part_1/whole"};

  EXPECT_EQ(
      moveit_planning_service::GraspPlanningPipeline::ResolveTargetObjectId(
          scene_ids, "table"),
      "table");
  EXPECT_EQ(
      moveit_planning_service::GraspPlanningPipeline::ResolveTargetObjectId(
          scene_ids, "bin"),
      "bin");
}

TEST_F(GraspPlanningPipelineTest, ResolveTargetObjectIdLeadingSlash) {
  const std::vector<std::string> scene_ids = {"table", "bin", "part_1/whole"};

  EXPECT_EQ(
      moveit_planning_service::GraspPlanningPipeline::ResolveTargetObjectId(
          scene_ids, "/table"),
      "table");
  EXPECT_EQ(
      moveit_planning_service::GraspPlanningPipeline::ResolveTargetObjectId(
          scene_ids, "/bin"),
      "bin");
}

TEST_F(GraspPlanningPipelineTest, ResolveTargetObjectIdCanonicalWhole) {
  const std::vector<std::string> scene_ids = {"table", "part_1/whole"};

  // Canonical naming convention: target "part_1" resolves to "part_1/whole"
  EXPECT_EQ(
      moveit_planning_service::GraspPlanningPipeline::ResolveTargetObjectId(
          scene_ids, "part_1"),
      "part_1/whole");
  EXPECT_EQ(
      moveit_planning_service::GraspPlanningPipeline::ResolveTargetObjectId(
          scene_ids, "/part_1"),
      "part_1/whole");
}

TEST_F(GraspPlanningPipelineTest, ResolveTargetObjectIdPrefixMatch) {
  const std::vector<std::string> scene_ids = {"table",
                                              "assembly_fixture/link_a"};

  EXPECT_EQ(
      moveit_planning_service::GraspPlanningPipeline::ResolveTargetObjectId(
          scene_ids, "assembly_fixture"),
      "assembly_fixture/link_a");
}

TEST_F(GraspPlanningPipelineTest, ResolveTargetObjectIdFallback) {
  const std::vector<std::string> scene_ids = {"table", "bin"};

  // Not in scene: returns original ID
  EXPECT_EQ(
      moveit_planning_service::GraspPlanningPipeline::ResolveTargetObjectId(
          scene_ids, "unknown_object"),
      "unknown_object");
  EXPECT_EQ(
      moveit_planning_service::GraspPlanningPipeline::ResolveTargetObjectId(
          scene_ids, ""),
      "");
}

TEST_F(GraspPlanningPipelineTest, GetPlanningSceneObjectIdsUnavailableService) {
  // Service is not running, so call should time out gracefully and return empty
  // list
  const auto object_ids =
      pipeline_->GetPlanningSceneObjectIds(std::chrono::milliseconds(50));
  EXPECT_TRUE(object_ids.empty());
}

TEST_F(GraspPlanningPipelineTest,
       GetPlanningSceneCollisionObjectUnavailableService) {
  // Service is not running, so call should time out gracefully and return
  // nullopt
  const auto obj = pipeline_->GetPlanningSceneCollisionObject(
      "test_object", std::chrono::milliseconds(50));
  EXPECT_FALSE(obj.has_value());
}

TEST_F(GraspPlanningPipelineTest, PlanGraspsRejectsEmptyTargetId) {
  auto request =
      std::make_shared<moveit_planning_interfaces::srv::PlanGrasps::Request>();
  request->target.id = "";
  request->group_name = "arm";

  auto response =
      std::make_shared<moveit_planning_interfaces::srv::PlanGrasps::Response>();

  pipeline_->PlanGrasps(request, response, std::chrono::milliseconds(50));

  EXPECT_EQ(response->error_code.val,
            moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME);
  EXPECT_TRUE(response->grasps.empty());
}

TEST_F(GraspPlanningPipelineTest, GetRobotModelReturnsNulloptWhenUnconfigured) {
  // In an unconfigured test node with no robot_description, GetRobotModel
  // returns nullptr gracefully
  EXPECT_EQ(pipeline_->GetRobotModel(), nullptr);
}

TEST_F(GraspPlanningPipelineTest,
       PlanGraspsFailsGracefullyWhenRobotModelUnavailable) {
  auto request =
      std::make_shared<moveit_planning_interfaces::srv::PlanGrasps::Request>();
  request->target.id = "test_box";
  request->group_name = "arm";
  request->obj_dims_in_meters.x = 0.1;
  request->obj_dims_in_meters.y = 0.1;
  request->obj_dims_in_meters.z = 0.1;

  auto response =
      std::make_shared<moveit_planning_interfaces::srv::PlanGrasps::Response>();

  pipeline_->PlanGrasps(request, response, std::chrono::milliseconds(50));

  EXPECT_EQ(response->error_code.val,
            moveit_msgs::msg::MoveItErrorCodes::FAILURE);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  int success = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return success;
}
