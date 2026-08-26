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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Whether to run in headless mode. Defaults to true (disables rviz_http_proxy and enables status_monitor).",
            ),
            DeclareLaunchArgument(
                "start_service_status_monitor",
                default_value="true",
                description="Whether to start the service status monitor. Defaults to true, which starts reporting service status to Flowstate",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "enable_visual_meshes",
                default_value="false",
                description="Whether to fetch and publish visual meshes. Defaults to false, so they will not be published as ROS visualization markers",
            ),
            Node(
                package="moveit_flowstate_ros_bridge",
                executable="moveit_flowstate_ros_bridge_main",
                name="flowstate_ros_bridge",
                output="both",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "enable_visual_meshes": LaunchConfiguration("enable_visual_meshes"),
                    }
                ],
            ),
            Node(
                package="moveit_flowstate_ros_bridge",
                executable="rviz_http_proxy",
                name="rviz_http_proxy",
                output="log",
                parameters=[
                    {
                        "http_port": 8123,
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("headless")),
            ),
            # Status monitor node providing Flowstate ServiceState gRPC server
            Node(
                package="moveit_flowstate_ros_bridge",
                executable="status_monitor",
                name="status_monitor",
                respawn=True,
                output="both",
                parameters=[
                    {
                        "bridge_lifecycle_node": "/flowstate_ros_bridge",
                    }
                ],
                condition=IfCondition(LaunchConfiguration("start_service_status_monitor")),
            ),
        ]
    )
