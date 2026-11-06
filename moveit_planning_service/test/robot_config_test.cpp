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

#include "moveit_planning_service/robot_config.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <string_view>

namespace moveit_planning_service {
namespace {

TEST(RobotConfigTest, TrimAndNormalizeRobotConfig) {
  EXPECT_EQ(TrimAndNormalizeRobotConfig(""), "");
  EXPECT_EQ(TrimAndNormalizeRobotConfig("   \t\r\n "), "");
  EXPECT_EQ(TrimAndNormalizeRobotConfig("omts"), "omts");
  EXPECT_EQ(TrimAndNormalizeRobotConfig("  OMTS\n"), "omts");
  EXPECT_EQ(TrimAndNormalizeRobotConfig("lab-bb-01"), "lab_bb_01");
  EXPECT_EQ(TrimAndNormalizeRobotConfig("  LAB-BB-01 "), "lab_bb_01");
}

TEST(RobotConfigTest, SupportedAllowlistAndDiagnosticString) {
  EXPECT_TRUE(IsSupportedRobotConfig("omts"));
  EXPECT_TRUE(IsSupportedRobotConfig("lab_bb_01"));
  EXPECT_FALSE(IsSupportedRobotConfig(""));
  EXPECT_FALSE(IsSupportedRobotConfig("lab-bb-01"));
  EXPECT_FALSE(IsSupportedRobotConfig("ur10e"));
  EXPECT_EQ(SupportedRobotConfigsString(), "omts, lab_bb_01");
}

TEST(RobotConfigTest, ResolveDefaultsToOmtsWhenUnsetOrBlank) {
  const auto unset = ResolveRobotConfig("", std::nullopt);
  EXPECT_EQ(unset.resolved_config, "omts");
  EXPECT_TRUE(unset.is_supported);
  EXPECT_FALSE(unset.param_overrides_proto);

  const auto blank = ResolveRobotConfig("   ", std::string_view(" \t "));
  EXPECT_EQ(blank.resolved_config, "omts");
  EXPECT_TRUE(blank.is_supported);
  EXPECT_FALSE(blank.param_overrides_proto);
}

TEST(RobotConfigTest, ResolveUsesProtoWhenParamEmpty) {
  const auto res = ResolveRobotConfig("", std::string_view("lab-bb-01"));
  EXPECT_EQ(res.resolved_config, "lab_bb_01");
  EXPECT_EQ(res.normalized_param, "");
  EXPECT_EQ(res.normalized_proto, "lab_bb_01");
  EXPECT_TRUE(res.is_supported);
  EXPECT_FALSE(res.param_overrides_proto);
}

TEST(RobotConfigTest, ResolveDetectsParamVsProtoMismatchWithoutFalsePositives) {
  // Equivalent after normalization -> no mismatch warning
  const auto matching =
      ResolveRobotConfig("lab-bb-01", std::string_view("LAB_BB_01"));
  EXPECT_EQ(matching.resolved_config, "lab_bb_01");
  EXPECT_TRUE(matching.is_supported);
  EXPECT_FALSE(matching.param_overrides_proto);

  // Parameter ("omts") differs from runtime_config.pb ("lab_bb_01") -> flagged
  const auto mismatch =
      ResolveRobotConfig("omts", std::string_view("lab-bb-01"));
  EXPECT_EQ(mismatch.resolved_config, "omts");
  EXPECT_EQ(mismatch.normalized_param, "omts");
  EXPECT_EQ(mismatch.normalized_proto, "lab_bb_01");
  EXPECT_TRUE(mismatch.is_supported);
  EXPECT_TRUE(mismatch.param_overrides_proto);
}

TEST(RobotConfigTest, ResolveRejectsUnsupportedValues) {
  const auto bad_param = ResolveRobotConfig("ur10e", std::string_view("omts"));
  EXPECT_EQ(bad_param.resolved_config, "ur10e");
  EXPECT_FALSE(bad_param.is_supported);
  EXPECT_TRUE(bad_param.param_overrides_proto);

  const auto bad_proto = ResolveRobotConfig("", std::string_view("../omts"));
  EXPECT_EQ(bad_proto.resolved_config, "../omts");
  EXPECT_FALSE(bad_proto.is_supported);
  EXPECT_FALSE(bad_proto.param_overrides_proto);
}

}  // namespace
}  // namespace moveit_planning_service
