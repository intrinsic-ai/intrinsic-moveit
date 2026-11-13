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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PKG_NAME = 'robot_hardware_descriptions'
SUPPORTED_ROBOT_CONFIGS = ('omts', 'lab_bb_01')


def build_robot_description_param() -> ParameterValue:
    """Build the ParameterValue rendering the selected robot_config Xacro."""
    xacro_file = PathJoinSubstitution([
        FindPackageShare(PKG_NAME),
        'urdf',
        [LaunchConfiguration('robot_config'), '.urdf.xacro'],
    ])
    return ParameterValue(Command(['xacro "', xacro_file, '"']), value_type=str)


def generate_launch_description():
    """Generate launch description for interactive URDF visualization."""
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(PKG_NAME), 'rviz', 'display.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config',
            default_value='omts',
            choices=list(SUPPORTED_ROBOT_CONFIGS),
            description="Robot configuration to visualize ('omts' or 'lab_bb_01').",
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': build_robot_description_param()}],
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
    ])
