#!/bin/bash

# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if [ ! -d "src/sdk-ros" ]; then
  echo "This script must be run at the top of a Colcon workspace with sdk-ros."
  exit
fi

ROS_DISTRO="jazzy"

show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Build and bundle the model container image for Flowstate."
  echo ""
  echo "Options:"
  echo "  -h, --help           Show this help message and exit"
  echo "  --ros_distro ROS_DISTRO  Name of the ROS distro (default: jazzy)"
  echo ""
}

while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      show_help
      exit 0
      ;;
    --ros_distro)
      ROS_DISTRO="$2"
      shift
      shift
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

set -o errexit
set -o verbose
src/sdk-ros/scripts/build_container.sh \
  --ros_distro "$ROS_DISTRO" \
  --service_name moveit_flowstate_ros_bridge \
  --service_package moveit_flowstate_ros_bridge \
  --dependencies nlohmann-json3-dev
src/sdk-ros/scripts/build_bundle.sh \
  --service_name moveit_flowstate_ros_bridge \
  --service_package moveit_flowstate_ros_bridge \
  --manifest_path src/sdk-ros/flowstate_ros_bridge/flowstate_ros_bridge.manifest.textproto \
  --default_config src/sdk-ros/flowstate_ros_bridge/flowstate_ros_bridge_default_config.pbtxt
