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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from moveit_planning_service.robot_config import DEFAULT_ROBOT_CONFIG
from moveit_planning_service.robot_config import normalize_robot_config


def _launch_setup(context):
    resolved_config = normalize_robot_config(
        LaunchConfiguration("robot_config").perform(context)
    )
    moveit_config = MoveItConfigsBuilder(
        resolved_config, package_name=f"{resolved_config}_moveit_config"
    ).to_moveit_configs()

    demo_launch = generate_demo_launch(moveit_config)

    planning_service = Node(
        package="moveit_planning_service",
        executable="moveit_planning_node",
        name="moveit_planning_node",
        parameters=[
            moveit_config.to_dict(),
            {"robot_config": resolved_config},
        ],
        output="both",
    )

    return [*demo_launch.entities, planning_service]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config",
                default_value=DEFAULT_ROBOT_CONFIG,
                description=(
                    "Workcell robot configuration to load ('omts' or 'lab_bb_01'; "
                    "'lab-bb-01' is also accepted)."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
