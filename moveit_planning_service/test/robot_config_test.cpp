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
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>

#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "moveit_planning_service.pb.h"

namespace moveit_planning_service {
namespace {

class ScopedTempDir {
 public:
  ScopedTempDir() {
    path_ = std::filesystem::temp_directory_path() /
            ("robot_config_test_" + std::to_string(::getpid()) + "_" +
             std::to_string(counter_++));
    std::filesystem::create_directories(path_);
  }

  ~ScopedTempDir() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  inline static int counter_ = 0;
  std::filesystem::path path_;
};

void WriteBytes(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(ofs.is_open());
  ofs.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  ASSERT_TRUE(ofs.good());
}

TEST(RobotConfigTest, TrimAndNormalizeRobotConfig) {
  EXPECT_EQ(TrimAndNormalizeRobotConfig(""), "");
  EXPECT_EQ(TrimAndNormalizeRobotConfig("   \t\r\n\f\v "), "");
  EXPECT_EQ(TrimAndNormalizeRobotConfig("omts"), "omts");
  EXPECT_EQ(TrimAndNormalizeRobotConfig("  OMTS\n\f\v"), "omts");
  EXPECT_EQ(TrimAndNormalizeRobotConfig("lab_bb_01"), "lab_bb_01");
  EXPECT_EQ(TrimAndNormalizeRobotConfig(" \tLAB_BB_01\r\n "), "lab_bb_01");
  // Hyphenated names are not rewritten to underscores.
  EXPECT_EQ(TrimAndNormalizeRobotConfig("lab-bb-01"), "lab-bb-01");
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

  const auto blank = ResolveRobotConfig("   ", std::string_view(" \t\f\v "));
  EXPECT_EQ(blank.resolved_config, "omts");
  EXPECT_TRUE(blank.is_supported);
  EXPECT_FALSE(blank.param_overrides_proto);
}

TEST(RobotConfigTest, ResolveUsesProtoWhenParamEmpty) {
  const auto res = ResolveRobotConfig("", std::string_view("lab_bb_01"));
  EXPECT_EQ(res.resolved_config, "lab_bb_01");
  EXPECT_EQ(res.normalized_param, "");
  EXPECT_EQ(res.normalized_proto, "lab_bb_01");
  EXPECT_TRUE(res.is_supported);
  EXPECT_FALSE(res.param_overrides_proto);
}

TEST(RobotConfigTest, ResolveDetectsParamVsProtoMismatchWithoutFalsePositives) {
  // Equivalent after case/whitespace normalization -> no mismatch warning
  const auto matching =
      ResolveRobotConfig(" lab_bb_01 ", std::string_view("LAB_BB_01"));
  EXPECT_EQ(matching.resolved_config, "lab_bb_01");
  EXPECT_TRUE(matching.is_supported);
  EXPECT_FALSE(matching.param_overrides_proto);

  // Parameter ("omts") differs from runtime_config.pb ("lab_bb_01") -> flagged
  const auto mismatch =
      ResolveRobotConfig("omts", std::string_view("lab_bb_01"));
  EXPECT_EQ(mismatch.resolved_config, "omts");
  EXPECT_EQ(mismatch.normalized_param, "omts");
  EXPECT_EQ(mismatch.normalized_proto, "lab_bb_01");
  EXPECT_TRUE(mismatch.is_supported);
  EXPECT_TRUE(mismatch.param_overrides_proto);
}

TEST(RobotConfigTest, ResolveRejectsUnsupportedValuesIncludingHyphenated) {
  const auto bad_param = ResolveRobotConfig("ur10e", std::string_view("omts"));
  EXPECT_EQ(bad_param.resolved_config, "ur10e");
  EXPECT_FALSE(bad_param.is_supported);
  EXPECT_TRUE(bad_param.param_overrides_proto);

  const auto hyphenated =
      ResolveRobotConfig("lab-bb-01", std::string_view("omts"));
  EXPECT_EQ(hyphenated.resolved_config, "lab-bb-01");
  EXPECT_FALSE(hyphenated.is_supported);
  EXPECT_TRUE(hyphenated.param_overrides_proto);

  const auto bad_proto = ResolveRobotConfig("", std::string_view("../omts"));
  EXPECT_EQ(bad_proto.resolved_config, "../omts");
  EXPECT_FALSE(bad_proto.is_supported);
  EXPECT_FALSE(bad_proto.param_overrides_proto);
}

TEST(RobotConfigTest, LoadRuntimeServiceConfigSilentFallbacks) {
  ScopedTempDir tmp;
  intrinsic::MoveItPlanningServiceConfig cfg;

  // 1. Empty path or missing file -> silent fallback (no error)
  const auto empty_path = LoadRuntimeServiceConfig("", &cfg);
  EXPECT_FALSE(empty_path.has_service_config);
  EXPECT_EQ(empty_path.error_message, "");

  const auto missing =
      LoadRuntimeServiceConfig(tmp.path() / "does_not_exist.pb", &cfg);
  EXPECT_FALSE(missing.has_service_config);
  EXPECT_EQ(missing.error_message, "");

  // 2. 0-byte file -> silent fallback
  const auto zero_pb = tmp.path() / "empty.pb";
  WriteBytes(zero_pb, "");
  const auto zero_res = LoadRuntimeServiceConfig(zero_pb, &cfg);
  EXPECT_FALSE(zero_res.has_service_config);
  EXPECT_EQ(zero_res.error_message, "");

  // 3. Unrelated Any type_url -> silent fallback
  intrinsic_proto::config::RuntimeContext unrelated_ctx;
  unrelated_ctx.mutable_config()->set_type_url(
      "type.googleapis.com/other.ServiceConfig");
  unrelated_ctx.mutable_config()->set_value("");
  const auto unrelated_pb = tmp.path() / "unrelated.pb";
  WriteBytes(unrelated_pb, unrelated_ctx.SerializeAsString());
  const auto unrelated_res = LoadRuntimeServiceConfig(unrelated_pb, &cfg);
  EXPECT_FALSE(unrelated_res.has_service_config);
  EXPECT_EQ(unrelated_res.error_message, "");
}

TEST(RobotConfigTest, LoadRuntimeServiceConfigUnpacksValidConfig) {
  ScopedTempDir tmp;
  intrinsic::MoveItPlanningServiceConfig expected;
  expected.set_robot_config("lab_bb_01");
  expected.set_zenoh_router_address("tcp/router:7447");

  intrinsic_proto::config::RuntimeContext ctx;
  ASSERT_TRUE(ctx.mutable_config()->PackFrom(expected));

  const auto valid_pb = tmp.path() / "valid.pb";
  WriteBytes(valid_pb, ctx.SerializeAsString());

  intrinsic::MoveItPlanningServiceConfig actual;
  const auto res = LoadRuntimeServiceConfig(valid_pb, &actual);
  EXPECT_TRUE(res.has_service_config);
  EXPECT_EQ(res.error_message, "");
  EXPECT_EQ(actual.robot_config(), "lab_bb_01");
  EXPECT_EQ(actual.zenoh_router_address(), "tcp/router:7447");
}

TEST(RobotConfigTest, LoadRuntimeServiceConfigFailsClosedOnErrors) {
  ScopedTempDir tmp;
  intrinsic::MoveItPlanningServiceConfig cfg;

  // 1. Directory instead of file -> error_message populated
  const auto dir_path = tmp.path() / "not_a_file.pb";
  std::filesystem::create_directory(dir_path);
  const auto dir_res = LoadRuntimeServiceConfig(dir_path, &cfg);
  EXPECT_FALSE(dir_res.has_service_config);
  EXPECT_NE(dir_res.error_message, "");

  // 2. Corrupt RuntimeContext bytes -> error_message populated
  const auto corrupt_pb = tmp.path() / "corrupt.pb";
  WriteBytes(corrupt_pb, std::string("\xff\x00\xfe\x01invalid", 11));
  const auto corrupt_res = LoadRuntimeServiceConfig(corrupt_pb, &cfg);
  EXPECT_FALSE(corrupt_res.has_service_config);
  EXPECT_NE(corrupt_res.error_message.find("Failed to decode RuntimeContext"),
            std::string::npos);

  // 3. Corrupt MoveItPlanningServiceConfig inside Any -> error_message
  // populated
  intrinsic_proto::config::RuntimeContext corrupt_any_ctx;
  corrupt_any_ctx.mutable_config()->set_type_url(
      "type.googleapis.com/intrinsic.MoveItPlanningServiceConfig");
  corrupt_any_ctx.mutable_config()->set_value(
      std::string("\xff\x00\xfe\x01invalid", 11));
  const auto corrupt_any_pb = tmp.path() / "corrupt_any.pb";
  WriteBytes(corrupt_any_pb, corrupt_any_ctx.SerializeAsString());
  const auto corrupt_any_res = LoadRuntimeServiceConfig(corrupt_any_pb, &cfg);
  EXPECT_FALSE(corrupt_any_res.has_service_config);
  EXPECT_NE(corrupt_any_res.error_message.find(
                "Failed to unpack MoveItPlanningServiceConfig"),
            std::string::npos);
}

}  // namespace
}  // namespace moveit_planning_service
