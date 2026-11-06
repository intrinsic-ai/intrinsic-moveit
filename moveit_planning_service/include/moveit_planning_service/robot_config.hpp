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

#ifndef MOVEIT_PLANNING_SERVICE_ROBOT_CONFIG_HPP_
#define MOVEIT_PLANNING_SERVICE_ROBOT_CONFIG_HPP_

#include <array>
#include <optional>
#include <string>
#include <string_view>

namespace moveit_planning_service {

inline constexpr std::string_view kDefaultRobotConfig = "omts";

inline constexpr std::array<std::string_view, 2> kSupportedRobotConfigs = {
    "omts",
    "lab_bb_01",
};

struct RobotConfigResolution {
  std::string resolved_config;
  std::string normalized_param;
  std::string normalized_proto;
  bool is_supported = false;
  bool param_overrides_proto = false;
};

// Strips ASCII whitespace, lowercases, and replaces '-' with '_'.
std::string TrimAndNormalizeRobotConfig(std::string_view raw);

// Returns true if `normalized` matches one of `kSupportedRobotConfigs`.
bool IsSupportedRobotConfig(std::string_view normalized);

// Returns comma-separated list of supported robot configs for diagnostics.
std::string SupportedRobotConfigsString();

// Resolves the active robot configuration with precedence:
// non-empty ROS parameter > non-empty runtime_config.pb > kDefaultRobotConfig.
// Sets `param_overrides_proto` when both inputs are non-empty and normalize to
// different values so callers can log a mismatch warning.
RobotConfigResolution ResolveRobotConfig(
    std::string_view param_robot_config,
    std::optional<std::string_view> proto_robot_config = std::nullopt);

}  // namespace moveit_planning_service

#endif  // MOVEIT_PLANNING_SERVICE_ROBOT_CONFIG_HPP_
