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

#include "moveit_plan_grasp_skill.h"

#include <gtest/gtest.h>

#include <string>

#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "moveit_plan_grasp_skill.pb.h"

namespace com::generic::skills::grasp {
namespace {

intrinsic_proto::world::ObjectReference MakeCandidate(
    const std::string& id = "test_object") {
  intrinsic_proto::world::ObjectReference candidate;
  candidate.set_id(id);
  return candidate;
}

TEST(MoveItPlanGraspSkillTest, BasicParametersMappedCorrectly) {
  GraspPlanningParams params;
  params.set_group_name("manipulator");
  params.set_end_effector_group("gripper");
  params.set_timeout_ms(5000.0);
  params.set_retract_dist_m(0.12);

  auto req = CreatePlanGraspsRequest(params, MakeCandidate());
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->group_name, "manipulator");
  EXPECT_EQ(req->end_effector_group, "gripper");
  EXPECT_DOUBLE_EQ(req->planning_timeout_sec, 5.0);
  EXPECT_DOUBLE_EQ(req->retract_dist_m, 0.12);
}

TEST(MoveItPlanGraspSkillTest, DefaultValuesAppliedCorrectly) {
  GraspPlanningParams params;
  // All optional fields left unset

  auto req = CreatePlanGraspsRequest(params, MakeCandidate());
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->group_name, "ur_manipulator");
  EXPECT_EQ(req->end_effector_group, "hand");
  EXPECT_DOUBLE_EQ(req->planning_timeout_sec, 10.0);
  EXPECT_DOUBLE_EQ(req->retract_dist_m, 0.1);
}

TEST(MoveItPlanGraspSkillTest, BoxGraspAnnotationsMappedCorrectly) {
  GraspPlanningParams params;
  auto* box = params.mutable_box_grasp_annotations();
  box->add_surfaces(0);
  box->add_surfaces(4);
  box->set_num_rotations(6);

  box->mutable_obj_dims_in_meters()->set_x(0.1);
  box->mutable_obj_dims_in_meters()->set_y(0.2);
  box->mutable_obj_dims_in_meters()->set_z(0.3);

  box->mutable_obj_t_obj_center()->mutable_position()->set_x(0.01);
  box->mutable_obj_t_obj_center()->mutable_position()->set_y(0.02);
  box->mutable_obj_t_obj_center()->mutable_position()->set_z(0.03);
  box->mutable_obj_t_obj_center()->mutable_orientation()->set_w(1.0);

  auto req = CreatePlanGraspsRequest(params, MakeCandidate());
  ASSERT_NE(req, nullptr);
  ASSERT_EQ(req->surfaces.size(), 2u);
  EXPECT_EQ(req->surfaces[0], 0);
  EXPECT_EQ(req->surfaces[1], 4);
  EXPECT_EQ(req->num_rotations, 6);

  EXPECT_DOUBLE_EQ(req->obj_dims_in_meters.x, 0.1);
  EXPECT_DOUBLE_EQ(req->obj_dims_in_meters.y, 0.2);
  EXPECT_DOUBLE_EQ(req->obj_dims_in_meters.z, 0.3);

  EXPECT_DOUBLE_EQ(req->obj_t_obj_center.position.x, 0.01);
  EXPECT_DOUBLE_EQ(req->obj_t_obj_center.position.y, 0.02);
  EXPECT_DOUBLE_EQ(req->obj_t_obj_center.position.z, 0.03);
  EXPECT_DOUBLE_EQ(req->obj_t_obj_center.orientation.w, 1.0);
}

TEST(MoveItPlanGraspSkillTest, ToolFrameById) {
  GraspPlanningParams params;
  params.mutable_tool_frame()->set_id("tool0");

  auto req = CreatePlanGraspsRequest(params, MakeCandidate());
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->tool_frame, "tool0");
}

TEST(MoveItPlanGraspSkillTest, ToolFrameByFrameName) {
  GraspPlanningParams params;
  params.mutable_tool_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name("gripper_tcp");
  params.mutable_tool_frame()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name("gripper");

  auto req = CreatePlanGraspsRequest(params, MakeCandidate());
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->tool_frame, "gripper_tcp");
}

TEST(MoveItPlanGraspSkillTest, ToolFrameByObjectName) {
  GraspPlanningParams params;
  params.mutable_tool_frame()
      ->mutable_by_name()
      ->mutable_object()
      ->set_object_name("gripper_tool");

  auto req = CreatePlanGraspsRequest(params, MakeCandidate());
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->tool_frame, "gripper_tool");
}

TEST(MoveItPlanGraspSkillTest, TargetObjectById) {
  GraspPlanningParams params;
  auto* candidate = params.add_candidate_objects();
  candidate->set_id("box_target");

  auto req = CreatePlanGraspsRequest(params, *candidate);
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->target.id, "box_target");
}

TEST(MoveItPlanGraspSkillTest, TargetObjectByName) {
  GraspPlanningParams params;
  auto* candidate = params.add_candidate_objects();
  candidate->mutable_by_name()->set_object_name("box_named_target");

  auto req = CreatePlanGraspsRequest(params, *candidate);
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->target.id, "box_named_target");
}

TEST(MoveItPlanGraspSkillTest, ExplicitCandidateEvaluatedWhenMultipleGiven) {
  GraspPlanningParams params;
  auto* cand1 = params.add_candidate_objects();
  cand1->set_id("primary_box");
  auto* cand2 = params.add_candidate_objects();
  cand2->set_id("secondary_box");

  auto req = CreatePlanGraspsRequest(params, params.candidate_objects(1));
  ASSERT_NE(req, nullptr);
  EXPECT_EQ(req->target.id, "secondary_box");
}

TEST(MoveItPlanGraspSkillTest, EmptyCandidateLeavesTargetUnset) {
  GraspPlanningParams params;
  intrinsic_proto::world::ObjectReference candidate;

  auto req = CreatePlanGraspsRequest(params, candidate);

  ASSERT_NE(req, nullptr);
  EXPECT_TRUE(req->target.id.empty());
}

TEST(MoveItPlanGraspSkillTest, AdvancedParamsSupportFields) {
  GraspPlanningParams params;
  params.set_plan_id("custom_plan_123");
  params.set_max_num_grasps(5);
  params.mutable_output_grasp_frame()->set_id("world/target_grasp");
  params.mutable_output_pregrasp_frame()->set_id("world/target_pregrasp");

  EXPECT_EQ(params.plan_id(), "custom_plan_123");
  EXPECT_EQ(params.max_num_grasps(), 5);
  EXPECT_EQ(params.output_grasp_frame().id(), "world/target_grasp");
  EXPECT_EQ(params.output_pregrasp_frame().id(), "world/target_pregrasp");
}

}  // namespace
}  // namespace com::generic::skills::grasp
