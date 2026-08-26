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

#ifndef MOVEIT_PLAN_MOTION_SKILL_H_
#define MOVEIT_PLAN_MOTION_SKILL_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

namespace com::generic::skills::motion {

class MotionPlanningSkill final : public intrinsic::skills::SkillInterface {
 public:
  static std::unique_ptr<intrinsic::skills::SkillInterface> CreateSkill();

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const intrinsic::skills::ExecuteRequest& request,
      intrinsic::skills::ExecuteContext& context) override;
};

}  // namespace com::generic::skills::motion

#endif  // MOVEIT_PLAN_MOTION_SKILL_H_
