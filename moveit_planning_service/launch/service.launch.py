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

"""Main MoveIt planning service launch file dispatching per-workcell launches."""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_LAUNCH_DIR = str(Path(__file__).resolve().parent)
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from planning_service_common import (  # noqa: E402
    declare_planning_service_arguments,
)

SUPPORTED_ROBOT_CONFIGS = ("omts", "lab_bb_01")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config",
                default_value="omts",
                choices=list(SUPPORTED_ROBOT_CONFIGS),
                description="Workcell robot configuration to load.",
            ),
            *declare_planning_service_arguments(),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("moveit_planning_service"),
                            "launch",
                            [LaunchConfiguration("robot_config"), ".launch.py"],
                        ]
                    )
                )
            ),
        ]
    )
