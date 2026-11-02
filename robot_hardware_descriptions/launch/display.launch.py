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

"""Launch RViz visualization for omts or lab_bb_01 robot hardware descriptions."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PKG_NAME = 'robot_hardware_descriptions'
SUPPORTED_ROBOT_CONFIGS = ('omts', 'lab_bb_01')


def resolve_display_xacro_path(robot_config: str) -> str:
    """Resolve and validate the top-level Xacro path for a robot_config name."""
    normalized = robot_config.strip().lower().replace('-', '_')
    if normalized not in SUPPORTED_ROBOT_CONFIGS:
        supported = ', '.join(SUPPORTED_ROBOT_CONFIGS)
        raise ValueError(
            f"Unsupported robot_config '{robot_config}'. "
            f'Supported configurations: {supported}'
        )
    pkg_share = get_package_share_directory(PKG_NAME)
    return os.path.join(pkg_share, 'urdf', f'{normalized}.urdf.xacro')


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    robot_config = LaunchConfiguration('robot_config').perform(context)
    xacro_file = resolve_display_xacro_path(robot_config)
    pkg_share = get_package_share_directory(PKG_NAME)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ]


def generate_launch_description():
    """Generate launch description for interactive URDF visualization."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config',
            default_value='omts',
            description=(
                "Robot configuration to visualize ('omts' or 'lab_bb_01'; "
                "'lab-bb-01' is also accepted)."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
