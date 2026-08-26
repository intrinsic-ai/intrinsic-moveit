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

# Set default RMW implementation before rclpy is imported
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')

import shutil
import socket
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
import launch
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
from moveit_msgs.msg import (
    CollisionObject,
    MoveItErrorCodes,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_planning_interfaces.srv import PlanGrasps
import pytest
import rclpy
from shape_msgs.msg import SolidPrimitive


def is_zenoh_router_running(port=7447):
    """Check if a Zenoh router is already listening on the given port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.2)
        return s.connect_ex(('127.0.0.1', port)) == 0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    actions = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp'),
    ]

    # If using rmw_zenoh_cpp (or default) and no router is running yet, spawn rmw_zenohd
    if shutil.which('ros2'):
        if not is_zenoh_router_running():
            actions.append(
                ExecuteProcess(
                    cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
                    output='screen',
                )
            )

    planning_service_share = get_package_share_directory('moveit_planning_service')
    service_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_service_share, 'launch', 'service.launch.py')
        ),
        launch_arguments={
            'headless': 'true',
            'use_mock_hardware': 'true',
            'expect_collision_objects': 'false',
            'start_service_status_monitor': 'false',
            'ros_service_call_timeout_sec': '5.0',
        }.items(),
    )
    actions.extend([
        service_launch,
        launch_testing.actions.ReadyToTest(),
    ])

    return launch.LaunchDescription(actions), {'service_launch': service_launch}


generate_test_description.__ready_to_test_action_timeout__ = 60.0


class TestGraspPlanningIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_grasp_planning_integration_client')

    def tearDown(self):
        self.node.destroy_node()

    def wait_for_service(self, client, timeout_sec=45.0):
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            if client.wait_for_service(timeout_sec=1.0):
                return True
        return False

    def call_service_sync(self, client, request, timeout_sec=15.0):
        future = client.call_async(request)
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                return future.result()
        return None

    def test_plan_grasps_for_collision_box(self, **kwargs):
        apply_scene_client = self.node.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        get_scene_client = self.node.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        plan_grasps_client = self.node.create_client(
            PlanGrasps, '/grasp_planning/plan_grasps'
        )

        # 1. Wait for required MoveIt and planning service endpoints
        self.assertTrue(
            self.wait_for_service(apply_scene_client, timeout_sec=45.0),
            'Timed out waiting for /apply_planning_scene service',
        )
        self.assertTrue(
            self.wait_for_service(get_scene_client, timeout_sec=45.0),
            'Timed out waiting for /get_planning_scene service',
        )
        self.assertTrue(
            self.wait_for_service(plan_grasps_client, timeout_sec=45.0),
            'Timed out waiting for /grasp_planning/plan_grasps service',
        )

        # 2. Add test block collision object to MoveIt planning scene
        apply_scene_req = ApplyPlanningScene.Request()
        apply_scene_req.scene.is_diff = True

        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = 'test_block'
        collision_object.operation = CollisionObject.ADD

        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        box_primitive.dimensions = [0.03, 0.03, 0.03]
        collision_object.primitives.append(box_primitive)

        box_pose = Pose()
        box_pose.position.x = 0.3
        box_pose.position.y = 0.0
        box_pose.position.z = 0.2
        box_pose.orientation.w = 1.0
        collision_object.primitive_poses.append(box_pose)

        apply_scene_req.scene.world.collision_objects.append(collision_object)

        apply_scene_resp = self.call_service_sync(
            apply_scene_client, apply_scene_req, timeout_sec=20.0
        )
        self.assertIsNotNone(
            apply_scene_resp, 'No response received from /apply_planning_scene'
        )
        self.assertTrue(
            apply_scene_resp.success,
            'Failed to apply collision object to planning scene',
        )

        # Verify collision object is registered in the planning scene
        object_registered = False
        start_wait = time.time()
        while rclpy.ok() and (time.time() - start_wait) < 10.0:
            get_scene_req = GetPlanningScene.Request()
            get_scene_req.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )
            get_scene_resp = self.call_service_sync(
                get_scene_client, get_scene_req, timeout_sec=2.0
            )
            if get_scene_resp is not None:
                for obj in get_scene_resp.scene.world.collision_objects:
                    if obj.id == 'test_block':
                        object_registered = True
                        break
            if object_registered:
                break
            time.sleep(0.2)

        self.assertTrue(
            object_registered,
            "Collision object 'test_block' not registered in planning scene",
        )

        # 3. Call grasp planning service for the target block
        plan_grasps_req = PlanGrasps.Request()
        plan_grasps_req.group_name = 'ur_manipulator'
        plan_grasps_req.end_effector_group = 'hand'
        plan_grasps_req.tool_frame = 'hande_tcp'
        plan_grasps_req.planning_timeout_sec = 10.0
        plan_grasps_req.retract_dist_m = 0.1
        plan_grasps_req.surfaces = [0, 1, 2, 3, 4, 5]
        plan_grasps_req.num_rotations = 4
        plan_grasps_req.target.id = 'test_block'

        plan_grasps_resp = self.call_service_sync(
            plan_grasps_client, plan_grasps_req, timeout_sec=15.0
        )
        self.assertIsNotNone(
            plan_grasps_resp, 'No response received from /grasp_planning/plan_grasps'
        )

        # 4. Verify planning results
        self.assertEqual(
            plan_grasps_resp.error_code.val,
            MoveItErrorCodes.SUCCESS,
            f'Grasp planning failed with error code: {plan_grasps_resp.error_code.val}',
        )
        self.assertGreater(
            len(plan_grasps_resp.grasps),
            0,
            'Expected at least one grasp candidate to be generated',
        )
        self.assertEqual(
            len(plan_grasps_resp.grasps),
            len(plan_grasps_resp.pre_grasp_poses),
            'Number of grasps and pre-grasp poses should match',
        )

        # Validate candidate posture structures
        expected_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        first_grasp = plan_grasps_resp.grasps[0]
        self.assertEqual(
            first_grasp.grasp_posture.joint_names,
            expected_joints,
            'Grasp posture joint names do not match expected manipulator joints',
        )
        self.assertEqual(
            first_grasp.pre_grasp_posture.joint_names,
            expected_joints,
            'Pre-grasp posture joint names do not match expected manipulator joints',
        )
        self.assertGreater(
            len(first_grasp.grasp_posture.points),
            0,
            'Grasp posture should contain trajectory points',
        )
        self.assertGreater(
            len(first_grasp.pre_grasp_posture.points),
            0,
            'Pre-grasp posture should contain trajectory points',
        )
        self.assertGreater(
            first_grasp.pre_grasp_approach.desired_distance,
            0.0,
            'Pre-grasp approach desired distance should be positive',
        )
        self.assertGreater(
            first_grasp.post_grasp_retreat.desired_distance,
            0.0,
            'Post-grasp retreat desired distance should be positive',
        )
