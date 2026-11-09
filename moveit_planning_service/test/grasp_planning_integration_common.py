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

"""Shared MoveIt grasp-planning launch integration test harness."""

import os
import shutil
import socket
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
import launch
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
from moveit_planning_interfaces.action import PlanGrasps
from moveit_planning_interfaces.msg import GraspCandidate
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


def is_zenoh_router_running(port: int = 7447) -> bool:
    """Check if a Zenoh router is already listening on the given port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.2)
        return sock.connect_ex(('127.0.0.1', port)) == 0


def build_grasp_planning_test_description(robot_config: str | None = None):
    """Build launch description for standalone mock-hardware grasp testing."""
    actions = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp'),
    ]

    if shutil.which('ros2') and not is_zenoh_router_running():
        actions.append(
            ExecuteProcess(
                cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
                output='screen',
            )
        )

    launch_args = {
        'headless': 'true',
        'use_mock_hardware': 'true',
        'expect_collision_objects': 'false',
        'start_service_status_monitor': 'false',
        'ros_service_call_timeout_sec': '5.0',
    }
    if robot_config is not None:
        launch_args['robot_config'] = robot_config

    planning_service_share = get_package_share_directory(
        'moveit_planning_service'
    )
    service_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_service_share, 'launch', 'service.launch.py')
        ),
        launch_arguments=launch_args.items(),
    )
    actions.extend(
        [
            service_launch,
            launch_testing.actions.ReadyToTest(),
        ]
    )

    return launch.LaunchDescription(actions), {'service_launch': service_launch}


class GraspPlanningIntegrationTestBase(unittest.TestCase):
    """Reusable end-to-end grasp planning verification across workcells."""

    EXPECTED_ROBOT_CONFIG: str = 'omts'
    EXPECTED_ROBOT_NAME: str = 'omts'
    EXPECTED_MESH_SUBDIR: str = 'package://robot_meshes_vendor/meshes/ur5e/'
    BOX_1_XYZ: tuple[float, float, float] = (0.3, -0.18, 0.015)
    BOX_2_XYZ: tuple[float, float, float] = (0.3, 0.18, 0.015)

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(
            f'test_grasp_planning_{self.EXPECTED_ROBOT_CONFIG}_client'
        )

    def tearDown(self):
        self.node.destroy_node()

    def wait_for_service(self, client, timeout_sec: float = 45.0) -> bool:
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            if client.wait_for_service(timeout_sec=1.0):
                return True
        return False

    def call_service_sync(self, client, request, timeout_sec: float = 15.0):
        future = client.call_async(request)
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                return future.result()
        return None

    def call_action_sync(
        self,
        client,
        goal,
        timeout_sec: float = 15.0,
        feedback_callback=None,
    ):
        goal_future = client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        rclpy.spin_until_future_complete(
            self.node, goal_future, timeout_sec=timeout_sec
        )
        if not goal_future.done():
            return None

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=timeout_sec
        )
        if not result_future.done():
            return None
        return result_future.result()

    def run_grasp_planning_for_collision_boxes(self):
        apply_scene_client = self.node.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        get_scene_client = self.node.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        get_params_client = self.node.create_client(
            GetParameters, '/moveit_planning_node/get_parameters'
        )
        rsp_params_client = self.node.create_client(
            GetParameters, '/robot_state_publisher/get_parameters'
        )
        plan_grasps_client = ActionClient(
            self.node, PlanGrasps, '/grasp_planning/plan_grasps'
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
            self.wait_for_service(get_params_client, timeout_sec=45.0),
            'Timed out waiting for /moveit_planning_node/get_parameters',
        )
        self.assertTrue(
            self.wait_for_service(rsp_params_client, timeout_sec=45.0),
            'Timed out waiting for /robot_state_publisher/get_parameters',
        )
        self.assertTrue(
            plan_grasps_client.wait_for_server(timeout_sec=45.0),
            'Timed out waiting for /grasp_planning/plan_grasps action server',
        )

        # Verify active robot_config and URDF loaded across the stack
        params_req = GetParameters.Request(names=['robot_config'])
        params_resp = self.call_service_sync(
            get_params_client, params_req, timeout_sec=15.0
        )
        self.assertIsNotNone(
            params_resp,
            'No response from /moveit_planning_node/get_parameters',
        )
        self.assertEqual(len(params_resp.values), 1)
        self.assertEqual(
            params_resp.values[0].string_value,
            self.EXPECTED_ROBOT_CONFIG,
        )

        rsp_resp = self.call_service_sync(
            rsp_params_client,
            GetParameters.Request(names=['robot_description']),
            timeout_sec=15.0,
        )
        self.assertIsNotNone(
            rsp_resp,
            'No response from /robot_state_publisher/get_parameters',
        )
        self.assertEqual(len(rsp_resp.values), 1)
        urdf_xml = rsp_resp.values[0].string_value
        self.assertIn(f'name="{self.EXPECTED_ROBOT_NAME}"', urdf_xml)
        self.assertIn(self.EXPECTED_MESH_SUBDIR, urdf_xml)

        # 2. Add two test block collision objects to MoveIt planning scene
        apply_scene_req = ApplyPlanningScene.Request()
        apply_scene_req.scene.is_diff = True

        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        box_primitive.dimensions = [0.03, 0.03, 0.03]

        for obj_id, (bx, by, bz) in (
            ('test_block', self.BOX_1_XYZ),
            ('test_block_2', self.BOX_2_XYZ),
        ):
            collision_object = CollisionObject()
            collision_object.header.frame_id = 'world'
            collision_object.id = obj_id
            collision_object.operation = CollisionObject.ADD
            collision_object.primitives.append(box_primitive)

            box_pose = Pose()
            box_pose.position.x = bx
            box_pose.position.y = by
            box_pose.position.z = bz
            box_pose.orientation.w = 1.0
            collision_object.primitive_poses.append(box_pose)
            apply_scene_req.scene.world.collision_objects.append(
                collision_object
            )

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

        expected_object_ids = {'test_block', 'test_block_2'}
        registered_object_ids = set()

        for _ in range(30):
            get_scene_req = GetPlanningScene.Request()
            get_scene_req.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_NAMES
            )
            get_scene_resp = self.call_service_sync(
                get_scene_client, get_scene_req, timeout_sec=5.0
            )
            if get_scene_resp is not None:
                registered_object_ids = {
                    obj.id
                    for obj in get_scene_resp.scene.world.collision_objects
                }
            if expected_object_ids.issubset(registered_object_ids):
                break
            time.sleep(0.2)

        self.assertTrue(
            expected_object_ids.issubset(registered_object_ids),
            'Collision objects were not registered in the planning scene',
        )

        # 3. Call grasp planning action for both candidate blocks
        plan_grasps_goal = PlanGrasps.Goal()
        plan_grasps_goal.group_name = 'ur_manipulator'
        plan_grasps_goal.end_effector_group = 'hand'
        plan_grasps_goal.tool_frame = 'hande_tcp'
        plan_grasps_goal.planning_timeout_sec = 10.0
        plan_grasps_goal.gripper_motion_duration_sec = 0.75
        plan_grasps_goal.retract_dist_m = 0.1
        plan_grasps_goal.surfaces = [0, 1, 2, 3, 4, 5]
        plan_grasps_goal.num_rotations = 4
        plan_grasps_goal.max_num_grasps = 20

        for obj_id in ('test_block', 'test_block_2'):
            candidate = GraspCandidate()
            candidate.target.id = obj_id
            plan_grasps_goal.candidates.append(candidate)

        feedback_messages = []
        action_response = self.call_action_sync(
            plan_grasps_client,
            plan_grasps_goal,
            timeout_sec=30.0,
            feedback_callback=lambda feedback_msg: feedback_messages.append(
                feedback_msg.feedback
            ),
        )
        self.assertIsNotNone(
            action_response,
            'No response received from /grasp_planning/plan_grasps action',
        )
        self.assertEqual(
            action_response.status,
            GoalStatus.STATUS_SUCCEEDED,
            f'Grasp planning action finished with status: {action_response.status}',
        )

        plan_grasps_result = action_response.result

        # 4. Verify planning results and action feedback
        self.assertEqual(
            plan_grasps_result.error_code.val,
            MoveItErrorCodes.SUCCESS,
            f'Grasp planning failed with error code: '
            f'{plan_grasps_result.error_code.val}',
        )
        self.assertEqual(
            len(feedback_messages),
            len(plan_grasps_goal.candidates),
            'Expected one feedback message per candidate object',
        )
        for index, feedback in enumerate(feedback_messages, start=1):
            self.assertEqual(
                feedback.completed_candidates,
                index,
                'Feedback completed_candidates did not increment sequentially',
            )
            self.assertEqual(
                feedback.total_candidates,
                len(plan_grasps_goal.candidates),
                'Feedback total_candidates does not match number of candidates',
            )

        self.assertGreater(
            len(plan_grasps_result.ranked_grasps),
            0,
            'Expected at least one ranked grasp to be generated',
        )
        self.assertLessEqual(
            len(plan_grasps_result.ranked_grasps),
            plan_grasps_goal.max_num_grasps,
            'Action returned more grasps than max_num_grasps',
        )

        grasp_qualities = [
            ranked_grasp.grasp.grasp_quality
            for ranked_grasp in plan_grasps_result.ranked_grasps
        ]
        self.assertEqual(
            grasp_qualities,
            sorted(grasp_qualities, reverse=True),
            'Expected ranked_grasps to be sorted by non-increasing grasp_quality',
        )

        expected_targets = {0: 'test_block', 1: 'test_block_2'}
        returned_candidate_indexes = set()
        for ranked_grasp in plan_grasps_result.ranked_grasps:
            self.assertIn(ranked_grasp.candidate_index, expected_targets)
            self.assertEqual(
                ranked_grasp.target_id,
                expected_targets[ranked_grasp.candidate_index],
            )
            returned_candidate_indexes.add(ranked_grasp.candidate_index)

        self.assertEqual(
            returned_candidate_indexes,
            set(expected_targets),
            'Expected valid grasps from both candidate objects',
        )

        first_ranked_grasp = plan_grasps_result.ranked_grasps[0]
        expected_gripper_joints = ['hande_left_finger_joint']
        expected_arm_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        first_grasp = first_ranked_grasp.grasp
        first_grasp_ik = first_ranked_grasp.grasp_ik_solution
        first_pregrasp_ik = first_ranked_grasp.pregrasp_ik_solution

        self.assertEqual(first_grasp_ik.name, expected_arm_joints)
        self.assertEqual(first_pregrasp_ik.name, expected_arm_joints)
        self.assertEqual(len(first_grasp_ik.position), len(expected_arm_joints))
        self.assertEqual(
            len(first_pregrasp_ik.position), len(expected_arm_joints)
        )

        self.assertEqual(
            first_grasp.grasp_posture.joint_names, expected_gripper_joints
        )
        self.assertEqual(
            first_grasp.pre_grasp_posture.joint_names, expected_gripper_joints
        )
        self.assertGreater(len(first_grasp.grasp_posture.points), 0)
        self.assertGreater(len(first_grasp.pre_grasp_posture.points), 0)
        self.assertEqual(
            len(first_grasp.pre_grasp_posture.points[0].positions), 1
        )
        self.assertEqual(len(first_grasp.grasp_posture.points[0].positions), 1)

        self.assertAlmostEqual(
            first_grasp.pre_grasp_posture.points[0].positions[0],
            -0.001,
            places=6,
        )
        self.assertAlmostEqual(
            first_grasp.grasp_posture.points[0].positions[0],
            0.025,
            places=6,
        )

        pre_grasp_duration = first_grasp.pre_grasp_posture.points[
            0
        ].time_from_start
        grasp_duration = first_grasp.grasp_posture.points[0].time_from_start
        self.assertAlmostEqual(
            pre_grasp_duration.sec + pre_grasp_duration.nanosec / 1e9,
            0.75,
            places=6,
        )
        self.assertAlmostEqual(
            grasp_duration.sec + grasp_duration.nanosec / 1e9,
            0.75,
            places=6,
        )
        self.assertGreater(first_grasp.pre_grasp_approach.desired_distance, 0.0)
        self.assertGreater(first_grasp.post_grasp_retreat.desired_distance, 0.0)
