# Copyright 2026 Intrinsic Innovation LLC

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    flowstate_ingress_address_arg = DeclareLaunchArgument(
        "flowstate_ingress_address",
        default_value="localhost:17080",
        description="Ingress gateway address for World, Geometry, and Executive services",
    )

    flowstate_zenoh_router_address_arg = DeclareLaunchArgument(
        "flowstate_zenoh_router_address",
        default_value="tcp/localhost:7447",
        description="Zenoh router address for TF and Robot State streaming",
    )

    robot_base_frame_id_arg = DeclareLaunchArgument(
        "robot_base_frame_id",
        default_value="ur_module/base_link",
        description="Base frame ID of the robot arm in the world scene",
    )

    robot_controller_instance_arg = DeclareLaunchArgument(
        "robot_controller_instance",
        default_value="icon",
        description="Robot controller instance name",
    )

    enable_robot_joint_state_topic_arg = DeclareLaunchArgument(
        "enable_robot_joint_state_topic",
        default_value="true",
        description="Whether to publish joint states from robot status",
    )

    robot_joint_state_topic_arg = DeclareLaunchArgument(
        "robot_joint_state_topic",
        default_value="/joint_states",
        description="Topic name for publishing robot joint states",
    )

    enable_force_torque_topic_arg = DeclareLaunchArgument(
        "enable_force_torque_topic",
        default_value="true",
        description="Whether to publish force/torque sensor data",
    )

    force_torque_topic_arg = DeclareLaunchArgument(
        "force_torque_topic",
        default_value="/fts_broadcaster/wrench",
        description="Topic name for publishing force/torque sensor wrench",
    )

    force_torque_sensor_frame_id_arg = DeclareLaunchArgument(
        "force_torque_sensor_frame_id",
        default_value="force_torque_sensor/force_torque_sensor/AtiForceTorqueSensor",
        description="Frame ID for the force/torque sensor",
    )

    throttle_robot_state_topic_arg = DeclareLaunchArgument(
        "throttle_robot_state_topic",
        default_value="false",
        description="Whether to subscribe to throttled robot status topic",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Whether to automatically configure and activate the lifecycle bridge node",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    bridge_node = Node(
        package="flowstate_ros_bridge",
        executable="flowstate_ros_bridge",
        name="flowstate_ros_bridge",
        output="both",
        parameters=[
            {
                "world_service_address": ParameterValue(
                    LaunchConfiguration("flowstate_ingress_address"),
                    value_type=str,
                ),
                "geometry_service_address": ParameterValue(
                    LaunchConfiguration("flowstate_ingress_address"),
                    value_type=str,
                ),
                "executive_service_address": ParameterValue(
                    LaunchConfiguration("flowstate_ingress_address"),
                    value_type=str,
                ),
                "skill_registry_address": ParameterValue(
                    LaunchConfiguration("flowstate_ingress_address"),
                    value_type=str,
                ),
                "solution_service_address": ParameterValue(
                    LaunchConfiguration("flowstate_ingress_address"),
                    value_type=str,
                ),
                "flowstate_zenoh_router_address": ParameterValue(
                    LaunchConfiguration("flowstate_zenoh_router_address"),
                    value_type=str,
                ),
                "bridge_plugins": [
                    "flowstate_ros_bridge::WorldBridge",
                    "flowstate_ros_bridge::ExecutiveBridge",
                ],
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart"),
                    value_type=bool,
                ),
                "enable_robot_joint_state_topic": ParameterValue(
                    LaunchConfiguration("enable_robot_joint_state_topic"),
                    value_type=bool,
                ),
                "robot_joint_state_topic": ParameterValue(
                    LaunchConfiguration("robot_joint_state_topic"),
                    value_type=str,
                ),
                "enable_force_torque_topic": ParameterValue(
                    LaunchConfiguration("enable_force_torque_topic"),
                    value_type=bool,
                ),
                "force_torque_topic": ParameterValue(
                    LaunchConfiguration("force_torque_topic"),
                    value_type=str,
                ),
                "robot_base_frame_id": ParameterValue(
                    LaunchConfiguration("robot_base_frame_id"),
                    value_type=str,
                ),
                "force_torque_sensor_frame_id": ParameterValue(
                    LaunchConfiguration("force_torque_sensor_frame_id"),
                    value_type=str,
                ),
                "robot_controller_instance": ParameterValue(
                    LaunchConfiguration("robot_controller_instance"),
                    value_type=str,
                ),
                "throttle_robot_state_topic": ParameterValue(
                    LaunchConfiguration("throttle_robot_state_topic"),
                    value_type=bool,
                ),
                "override_joint_names": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            flowstate_ingress_address_arg,
            flowstate_zenoh_router_address_arg,
            robot_base_frame_id_arg,
            robot_controller_instance_arg,
            enable_robot_joint_state_topic_arg,
            robot_joint_state_topic_arg,
            enable_force_torque_topic_arg,
            force_torque_topic_arg,
            force_torque_sensor_frame_id_arg,
            throttle_robot_state_topic_arg,
            autostart_arg,
            use_sim_time_arg,
            bridge_node,
        ]
    )
