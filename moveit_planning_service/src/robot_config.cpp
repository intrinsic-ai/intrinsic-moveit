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
#include <optional>
#include <string>
#include <string_view>

namespace moveit_planning_service {

std::string TrimAndNormalizeRobotConfig(std::string_view raw) {
  const auto first = raw.find_first_not_of(" \t\r\n");
  if (first == std::string_view::npos) {
    return "";
  }
  const auto last = raw.find_last_not_of(" \t\r\n");
  std::string normalized(raw.substr(first, last - first + 1));
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  std::replace(normalized.begin(), normalized.end(), '-', '_');
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

}  // namespace moveit_planning_service
