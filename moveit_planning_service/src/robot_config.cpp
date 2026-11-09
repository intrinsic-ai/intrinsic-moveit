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

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>

#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "moveit_planning_service.pb.h"

namespace moveit_planning_service {

namespace {
constexpr std::string_view kAsciiWhitespace = " \t\r\n\f\v";
}  // namespace

std::string TrimAndNormalizeRobotConfig(std::string_view raw) {
  const auto first = raw.find_first_not_of(kAsciiWhitespace);
  if (first == std::string_view::npos) {
    return "";
  }
  const auto last = raw.find_last_not_of(kAsciiWhitespace);
  std::string normalized(raw.substr(first, last - first + 1));
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return normalized;
}

bool IsSupportedRobotConfig(std::string_view normalized) {
  return std::find(kSupportedRobotConfigs.begin(), kSupportedRobotConfigs.end(),
                   normalized) != kSupportedRobotConfigs.end();
}

std::string SupportedRobotConfigsString() {
  std::string joined;
  for (std::size_t i = 0; i < kSupportedRobotConfigs.size(); ++i) {
    if (i > 0) {
      joined += ", ";
    }
    joined.append(kSupportedRobotConfigs[i]);
  }
  return joined;
}

RobotConfigResolution ResolveRobotConfig(
    std::string_view param_robot_config,
    std::optional<std::string_view> proto_robot_config) {
  RobotConfigResolution result;
  result.normalized_param = TrimAndNormalizeRobotConfig(param_robot_config);
  if (proto_robot_config.has_value()) {
    result.normalized_proto = TrimAndNormalizeRobotConfig(*proto_robot_config);
  }

  if (!result.normalized_param.empty()) {
    result.resolved_config = result.normalized_param;
    result.param_overrides_proto =
        !result.normalized_proto.empty() &&
        result.normalized_param != result.normalized_proto;
  } else if (!result.normalized_proto.empty()) {
    result.resolved_config = result.normalized_proto;
  } else {
    result.resolved_config = std::string(kDefaultRobotConfig);
  }

  result.is_supported = IsSupportedRobotConfig(result.resolved_config);
  return result;
}

RuntimeConfigLoadResult LoadRuntimeServiceConfig(
    const std::filesystem::path& runtime_config_path,
    intrinsic::MoveItPlanningServiceConfig* out_config) {
  RuntimeConfigLoadResult result;
  if (runtime_config_path.empty()) {
    return result;
  }

  std::error_code dir_ec;
  if (std::filesystem::is_directory(runtime_config_path, dir_ec)) {
    result.error_message = "Failed to read runtime config at '" +
                           runtime_config_path.string() + "': is a directory";
    return result;
  }

  std::ifstream ifs(runtime_config_path, std::ios::binary);
  if (!ifs.is_open()) {
    std::error_code exists_ec;
    const bool exists = std::filesystem::exists(runtime_config_path, exists_ec);
    if (!exists && !exists_ec) {
      return result;
    }
    result.error_message = "Failed to open runtime config at '" +
                           runtime_config_path.string() + "'" +
                           (exists_ec ? (": " + exists_ec.message()) : "");
    return result;
  }

  std::ostringstream oss;
  oss << ifs.rdbuf();
  if (ifs.bad()) {
    result.error_message = "Failed to read runtime config bytes from '" +
                           runtime_config_path.string() + "'";
    return result;
  }
  const std::string raw_bytes = oss.str();
  if (raw_bytes.empty()) {
    return result;
  }

  intrinsic_proto::config::RuntimeContext runtime_context;
  if (!runtime_context.ParseFromString(raw_bytes)) {
    result.error_message = "Failed to decode RuntimeContext from '" +
                           runtime_config_path.string() + "'";
    return result;
  }

  if (!runtime_context.has_config() ||
      runtime_context.config().type_url().empty()) {
    return result;
  }
  if (!runtime_context.config().Is<intrinsic::MoveItPlanningServiceConfig>()) {
    return result;
  }

  intrinsic::MoveItPlanningServiceConfig local_config;
  intrinsic::MoveItPlanningServiceConfig* target =
      (out_config != nullptr) ? out_config : &local_config;
  if (!runtime_context.config().UnpackTo(target)) {
    result.error_message =
        "Failed to unpack MoveItPlanningServiceConfig from '" +
        runtime_config_path.string() + "'";
    return result;
  }

  result.has_service_config = true;
  return result;
}

}  // namespace moveit_planning_service
