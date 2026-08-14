// Copyright 2026 Intrinsic Innovation LLC

#include "moveit_plan_motion_skill.h"

#include <gtest/gtest.h>

#include "moveit_plan_motion_skill.pb.h"

namespace com::generic::skills::motion {
namespace {

TEST(MoveItPlanMotionSkillTest, BasicParametersMappedCorrectly) {
  MotionPlanningParams params;
  params.set_group_name("manipulator");
  params.set_timeout_ms(5000.0);
  params.set_target_frame("world_target");
  params.set_tool_frame("robot_tcp");

  auto req = CreateMotionPlanRequest(params);
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->motion_plan_request.group_name, "manipulator");
  EXPECT_DOUBLE_EQ(req->motion_plan_request.allowed_planning_time, 5.0);

  ASSERT_EQ(req->motion_plan_request.goal_constraints.size(), 1u);
  const auto& goal_constraint = req->motion_plan_request.goal_constraints[0];

  ASSERT_EQ(goal_constraint.position_constraints.size(), 1u);
  EXPECT_EQ(goal_constraint.position_constraints[0].header.frame_id, "world_target");
  EXPECT_EQ(goal_constraint.position_constraints[0].link_name, "robot_tcp");

  ASSERT_EQ(goal_constraint.orientation_constraints.size(), 1u);
  EXPECT_EQ(goal_constraint.orientation_constraints[0].header.frame_id, "world_target");
  EXPECT_EQ(goal_constraint.orientation_constraints[0].link_name, "robot_tcp");
}

TEST(MoveItPlanMotionSkillTest, StartJointStateMappedCorrectly) {
  MotionPlanningParams params;
  params.set_group_name("manipulator");
  params.set_timeout_ms(5000.0);
  params.set_target_frame("world_target");
  params.set_tool_frame("robot_tcp");

  auto* start_state = params.mutable_start_joint_state();
  start_state->add_joints(0.1);
  start_state->add_joints(-0.2);
  start_state->add_joints(0.3);

  auto req = CreateMotionPlanRequest(params);
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->motion_plan_request.start_state.joint_state.position.size(), 3u);
  EXPECT_DOUBLE_EQ(req->motion_plan_request.start_state.joint_state.position[0], 0.1);
  EXPECT_DOUBLE_EQ(req->motion_plan_request.start_state.joint_state.position[1], -0.2);
  EXPECT_DOUBLE_EQ(req->motion_plan_request.start_state.joint_state.position[2], 0.3);
}

}  // namespace
}  // namespace com::generic::skills::motion
