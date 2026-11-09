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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_planning_service.robot_config import DEFAULT_RUNTIME_CONFIG_PATH
from moveit_planning_service.robot_config import resolve_robot_config


def build_planning_node_params(
    resolved_config: str, moveit_config_dict: dict
) -> list[dict]:
    return [
        moveit_config_dict,
        {
            "robot_config": resolved_config,
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


def build_service_launch_actions(
    context, runtime_config_path: str | None = None
):
    if runtime_config_path is None:
        runtime_config_path = DEFAULT_RUNTIME_CONFIG_PATH
    cli_robot_config = LaunchConfiguration("robot_config").perform(context)
    resolved_config = resolve_robot_config(
        cli_robot_config=cli_robot_config,
        runtime_config_path=runtime_config_path,
    )
    moveit_pkg_name = f"{resolved_config}_moveit_config"

    moveit_config = MoveItConfigsBuilder(
        resolved_config, package_name=moveit_pkg_name
    ).to_moveit_configs()

    planning_params = build_planning_node_params(
        resolved_config, moveit_config.to_dict()
    )

    moveit_planning_node = Node(
        package="moveit_planning_service",
        executable="moveit_planning_node",
        name="moveit_planning_node",
        respawn=True,
        output="both",
        parameters=planning_params,
    )

    robot_moveit_config_path = get_package_share_directory(moveit_pkg_name)

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
                robot_moveit_config_path,
                "launch",
                "spawn_controllers.launch.py",
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

    actions = [
        LogInfo(
            msg=(
                f"[service.launch.py] Loading robot_config='{resolved_config}' "
                f"from package '{moveit_pkg_name}'"
            )
        ),
        rsp_launch,
        move_group_launch,
        ros2_control_node,
        spawn_controllers_launch,
        moveit_planning_node,
        status_monitor_node,
        rviz_launch,
    ]
    if static_vj_launch is not None:
        actions.append(static_vj_launch)
    return actions


def generate_launch_description():
    robot_config_arg = DeclareLaunchArgument(
        "robot_config",
        default_value="",
        description=(
            "Workcell robot configuration to load ('omts' or 'lab_bb_01'). "
            "When empty, falls back to /etc/intrinsic/runtime_config.pb or "
            "defaults to 'omts'."
        ),
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Whether to run in headless mode. Defaults to true. If false, runs locally with rviz",
    )

    start_service_status_monitor_arg = DeclareLaunchArgument(
        "start_service_status_monitor",
        default_value="true",
        description="Whether to start the service status monitor. Defaults to true, which starts reporting service status to the platform",
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="If true, start mock ros2_control hardware (for offline testing without platform bridge)",
    )

    ros_service_call_timeout_sec_arg = DeclareLaunchArgument(
        "ros_service_call_timeout_sec",
        default_value="5.0",
        description="Timeout length in seconds for ROS 2 service calls",
    )

    add_collision_retry_interval_sec_arg = DeclareLaunchArgument(
        "add_collision_retry_interval_sec",
        default_value="2.0",
        description="Interval in seconds between retries to synchronize collision objects",
    )

    expect_collision_objects_arg = DeclareLaunchArgument(
        "expect_collision_objects",
        default_value="true",
        description="If true, require non-empty collision objects in MoveIt planning scene before marking node ready",
    )

    publish_world_root_tf_arg = DeclareLaunchArgument(
        "publish_world_root_tf",
        default_value="true",
        description="Whether to publish static transform from world to root in connected mode",
    )

    publish_robot_base_tf_arg = DeclareLaunchArgument(
        "publish_robot_base_tf",
        default_value="true",
        description="Whether to publish static transform from robot_base_frame_id to base_link in connected mode",
    )

    intrinsic_core_ingress_address_arg = DeclareLaunchArgument(
        "intrinsic_core_ingress_address",
        default_value="localhost:17080",
        description="Ingress gateway address for the World service (default localhost:17080)",
    )

    zenoh_router_address_arg = DeclareLaunchArgument(
        "zenoh_router_address",
        default_value="tcp/localhost:7447",
        description="Address of Zenoh router for local execution (default tcp/localhost:7447)",
    )

    world_sync_interval_sec_arg = DeclareLaunchArgument(
        "world_sync_interval_sec",
        default_value="2.0",
        description="Interval in seconds for periodic background synchronization of World objects",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("moveit_planning_service"),
            "rviz",
            "motion_planning_service.rviz",
        ),
        description="Path to RViz config file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    return LaunchDescription(
        [
            robot_config_arg,
            headless_arg,
            start_service_status_monitor_arg,
            rviz_config_arg,
            use_mock_hardware_arg,
            use_sim_time_arg,
            intrinsic_core_ingress_address_arg,
            zenoh_router_address_arg,
            ros_service_call_timeout_sec_arg,
            add_collision_retry_interval_sec_arg,
            expect_collision_objects_arg,
            publish_world_root_tf_arg,
            publish_robot_base_tf_arg,
            world_sync_interval_sec_arg,
            OpaqueFunction(function=build_service_launch_actions),
        ]
    )
