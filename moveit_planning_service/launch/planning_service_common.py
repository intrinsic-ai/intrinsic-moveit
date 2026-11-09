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

"""Shared MoveIt planning service launch builder parameterized by workcell."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def declare_planning_service_arguments() -> list[DeclareLaunchArgument]:
    """Declare standard launch arguments shared across all workcell launches."""
    return [
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Whether to run in headless mode. Defaults to true. If false, runs locally with rviz",
        ),
        DeclareLaunchArgument(
            "start_service_status_monitor",
            default_value="true",
            description="Whether to start the service status monitor. Defaults to true, which starts reporting service status to the platform",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("moveit_planning_service"),
                "rviz",
                "motion_planning_service.rviz",
            ),
            description="Path to RViz config file",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="If true, start mock ros2_control hardware (for offline testing without platform bridge)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "intrinsic_core_ingress_address",
            default_value="localhost:17080",
            description="Ingress gateway address for the World service (default localhost:17080)",
        ),
        DeclareLaunchArgument(
            "zenoh_router_address",
            default_value="tcp/localhost:7447",
            description="Address of Zenoh router for local execution (default tcp/localhost:7447)",
        ),
        DeclareLaunchArgument(
            "ros_service_call_timeout_sec",
            default_value="5.0",
            description="Timeout length in seconds for ROS 2 service calls",
        ),
        DeclareLaunchArgument(
            "add_collision_retry_interval_sec",
            default_value="2.0",
            description="Interval in seconds between retries to synchronize collision objects",
        ),
        DeclareLaunchArgument(
            "expect_collision_objects",
            default_value="true",
            description="If true, require non-empty collision objects in MoveIt planning scene before marking node ready",
        ),
        DeclareLaunchArgument(
            "publish_world_root_tf",
            default_value="true",
            description="Whether to publish static transform from world to root in connected mode",
        ),
        DeclareLaunchArgument(
            "publish_robot_base_tf",
            default_value="true",
            description="Whether to publish static transform from robot_base_frame_id to base_link in connected mode",
        ),
        DeclareLaunchArgument(
            "world_sync_interval_sec",
            default_value="2.0",
            description="Interval in seconds for periodic background synchronization of World objects",
        ),
    ]


def generate_robot_service_launch_description(
    robot_name: str, moveit_config_package: str
) -> LaunchDescription:
    """Build the MoveIt planning service LaunchDescription for a workcell."""
    moveit_config = MoveItConfigsBuilder(
        robot_name, package_name=moveit_config_package
    ).to_moveit_configs()

    planning_params = [
        moveit_config.to_dict(),
        {
            "intrinsic_core_ingress_address": ParameterValue(
                LaunchConfiguration("intrinsic_core_ingress_address"),
                value_type=str,
            ),
            "zenoh_router_address": ParameterValue(
                LaunchConfiguration("zenoh_router_address"),
                value_type=str,
            ),
            "ros_service_call_timeout_sec": ParameterValue(
                LaunchConfiguration("ros_service_call_timeout_sec"),
                value_type=float,
            ),
            "add_collision_retry_interval_sec": ParameterValue(
                LaunchConfiguration("add_collision_retry_interval_sec"),
                value_type=float,
            ),
            "expect_collision_objects": ParameterValue(
                LaunchConfiguration("expect_collision_objects"),
                value_type=bool,
            ),
            "use_mock_hardware": ParameterValue(
                LaunchConfiguration("use_mock_hardware"),
                value_type=bool,
            ),
            "publish_world_root_tf": ParameterValue(
                LaunchConfiguration("publish_world_root_tf"),
                value_type=bool,
            ),
            "publish_robot_base_tf": ParameterValue(
                LaunchConfiguration("publish_robot_base_tf"),
                value_type=bool,
            ),
            "world_sync_interval_sec": ParameterValue(
                LaunchConfiguration("world_sync_interval_sec"),
                value_type=float,
            ),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"),
                value_type=bool,
            ),
        },
    ]

    moveit_planning_node = Node(
        package="moveit_planning_service",
        executable="moveit_planning_node",
        name="moveit_planning_node",
        respawn=True,
        output="both",
        parameters=planning_params,
    )

    robot_moveit_config_path = get_package_share_directory(
        moveit_config_package
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_moveit_config_path, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_moveit_config_path, "launch", "move_group.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
    )

    static_vj_launch_path = os.path.join(
        robot_moveit_config_path, "launch", "static_virtual_joint_tfs.launch.py"
    )
    static_vj_launch = None
    if os.path.exists(static_vj_launch_path):
        static_vj_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_vj_launch_path),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_mock_hardware")),
        )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        condition=IfCondition(LaunchConfiguration("use_mock_hardware")),
        output="both",
    )

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_moveit_config_path, "launch", "spawn_controllers.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_mock_hardware")),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_moveit_config_path, "launch", "moveit_rviz.launch.py"
            )
        ),
        launch_arguments={
            "rviz_config": LaunchConfiguration("rviz_config")
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    status_monitor_node = Node(
        package="moveit_planning_service",
        executable="status_monitor",
        name="status_monitor",
        respawn=True,
        output="both",
        parameters=[
            {
                "ros_service_call_timeout_sec": ParameterValue(
                    LaunchConfiguration("ros_service_call_timeout_sec"),
                    value_type=float,
                ),
                "expect_collision_objects": ParameterValue(
                    LaunchConfiguration("expect_collision_objects"),
                    value_type=bool,
                ),
                "use_mock_hardware": ParameterValue(
                    LaunchConfiguration("use_mock_hardware"),
                    value_type=bool,
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
            },
        ],
        condition=IfCondition(
            LaunchConfiguration("start_service_status_monitor")
        ),
    )

    launch_actions = [
        *declare_planning_service_arguments(),
        rsp_launch,
        move_group_launch,
        ros2_control_node,
        spawn_controllers_launch,
        moveit_planning_node,
        status_monitor_node,
        rviz_launch,
    ]
    if static_vj_launch is not None:
        launch_actions.append(static_vj_launch)

    return LaunchDescription(launch_actions)
