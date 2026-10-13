#!/usr/bin/env python3

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

"""Benchmark sequential grasp planning for multiple candidate objects."""

import argparse
import statistics
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.client import Client
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from action_msgs.msg import GoalStatus

from moveit_planning_interfaces.action import PlanGrasps
from moveit_planning_interfaces.msg import GraspCandidate
from rclpy.action import ActionClient

DEFAULT_CANDIDATE_COUNTS = (1, 2, 4, 8, 10)
DEFAULT_REPETITIONS = 5


@dataclass(frozen=True)
class BenchmarkConfig:
    candidate_counts: tuple[int, ...]
    repetitions: int
    planning_timeout_sec: float


@dataclass(frozen=True)
class BatchMeasurement:
    candidate_count: int
    duration_sec: float
    successful_candidates: int
    grasp_count: int
    error_code: int


class GraspPlanningBenchmark(Node):

    def __init__(self) -> None:
        super().__init__("grasp_planning_benchmark")

        self.apply_planning_scene_client: Client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        self.get_scene_client: Client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

        self.plan_grasp_client = ActionClient(
            self, PlanGrasps, "/grasp_planning/plan_grasps"
        )

    def wait_for_services(self, timeout_sec: float = 30.0) -> bool:
        """Wait for all required services to be available."""

        clients: tuple[tuple[str, Client], ...] = (
            ("/apply_planning_scene", self.apply_planning_scene_client),
            ("/get_planning_scene", self.get_scene_client),
        )

        for service_name, client in clients:
            self.get_logger().info(f"Waiting for service {service_name}...")

            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(
                    f"Service {service_name} not available after {timeout_sec:.1f} seconds."
                )
                return False

        self.get_logger().info("Waiting for action /grasp_planning/plan_grasps...")

        if not self.plan_grasp_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(
                "Action /grasp_planning/plan_grasps "
                f"not available after {timeout_sec:.1f} seconds."
            )
            return False
        return True

    def create_collision_box(self, index: int) -> CollisionObject:
        """Create a deterministic collision box for one candidate."""

        columns = 4
        row: int = index // columns
        column: int = index % columns

        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = f"benchmark_box_{index}"
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.03, 0.03, 0.03]
        collision_object.primitives.append(primitive)

        pose = Pose()
        pose.position.x = 0.30 + (row * 0.12)
        pose.position.y = -0.18 + (column * 0.12)
        pose.position.z = 0.015
        pose.orientation.w = 1.0
        collision_object.primitive_poses.append(pose)

        return collision_object

    def add_collision_boxes(self, candidate_count: int, timeout_sec: float) -> bool:
        """Add all benchmark candidate boxes to the planning scene."""

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True

        for index in range(candidate_count):
            request.scene.world.collision_objects.append(
                self.create_collision_box(index)
            )

        response = self.call_service(
            self.apply_planning_scene_client, request, timeout_sec=timeout_sec
        )

        if response is None:
            return False

        if not response.success:
            self.get_logger().error("MoveIt rejected the planning scene update.")
            return False

        self.get_logger().info(
            f"Added {candidate_count} collision boxes to the planning scene."
        )

        return True

    def verify_collision_boxes(
        self, candidate_count: int, timeout_sec: float = 10.0
    ) -> bool:
        """Verify that all benchmark candidate boxes exist in the planning scene."""

        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY

        response = self.call_service(self.get_scene_client, request, timeout_sec)

        if response is None:
            return False

        expected_ids: set[str] = {
            f"benchmark_box_{index}" for index in range(candidate_count)
        }
        actual_ids = {
            collision_object.id
            for collision_object in response.scene.world.collision_objects
        }

        missing_ids = expected_ids - actual_ids
        if missing_ids:
            self.get_logger().error(
                "Objects missing from the planning scene: "
                + ", ".join(sorted(missing_ids))
            )
            return False

        self.get_logger().info(
            f"Verified {candidate_count} candidate objects in the planning scene."
        )

        return True

    def remove_existing_benchmark_boxes(
        self,
        timeout_sec: float = 10.0,
    ) -> bool:
        """Remove benchmark objects left by previous executions."""

        get_request = GetPlanningScene.Request()
        get_request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES

        get_response = self.call_service(
            self.get_scene_client,
            get_request,
            timeout_sec,
        )

        if get_response is None:
            return False

        benchmark_ids = [
            collision_object.id
            for collision_object in get_response.scene.world.collision_objects
            if collision_object.id.startswith("benchmark_box_")
        ]

        if not benchmark_ids:
            return True

        apply_request = ApplyPlanningScene.Request()
        apply_request.scene.is_diff = True

        for object_id in benchmark_ids:
            collision_object = CollisionObject()
            collision_object.id = object_id
            collision_object.operation = CollisionObject.REMOVE
            apply_request.scene.world.collision_objects.append(collision_object)

        apply_response = self.call_service(
            self.apply_planning_scene_client,
            apply_request,
            timeout_sec,
        )

        if apply_response is None or not apply_response.success:
            self.get_logger().error(
                "Failed to remove objects from a previous benchmark run."
            )
            return False

        self.get_logger().info(
            f"Removed {len(benchmark_ids)} objects from a previous benchmark run."
        )
        return True

    def create_grasp_candidate(self, index: int) -> GraspCandidate:
        """Create one grasp candidate for a collision object."""

        candidate = GraspCandidate()
        candidate.target.id = f"benchmark_box_{index}"

        candidate.obj_dims_in_meters.x = 0.03
        candidate.obj_dims_in_meters.y = 0.03
        candidate.obj_dims_in_meters.z = 0.03
        candidate.obj_t_obj_center.orientation.w = 1.0

        return candidate

    def create_plan_grasps_goal(
        self,
        candidate_count: int,
        planning_timeout_sec: float,
    ) -> PlanGrasps.Goal:

        goal = PlanGrasps.Goal()
        goal.group_name = "ur_manipulator"
        goal.end_effector_group = "hand"
        goal.tool_frame = "hande_tcp"
        goal.planning_timeout_sec = planning_timeout_sec
        goal.gripper_motion_duration_sec = 0.5
        goal.retract_dist_m = 0.1
        goal.surfaces = [0, 1, 2, 3, 4, 5]
        goal.num_rotations = 4
        goal.max_num_grasps = candidate_count * 10

        for index in range(candidate_count):
            goal.candidates.append(self.create_grasp_candidate(index))

        return goal

    def run_action_batch(
        self,
        candidate_count: int,
        planning_timeout_sec: float,
    ) -> BatchMeasurement:
        """Send one goal containing all candidates and measure its duration."""

        goal = self.create_plan_grasps_goal(
            candidate_count,
            planning_timeout_sec,
        )

        # The server divides planning_timeout_sec across all candidates.
        # Add a small communication margin for action overhead.
        action_timeout_sec = planning_timeout_sec + 5.0

        start_time = time.perf_counter()
        action_response = self.call_action(
            goal,
            timeout_sec=action_timeout_sec,
        )
        duration_sec = time.perf_counter() - start_time

        if action_response is None:
            return BatchMeasurement(
                candidate_count=candidate_count,
                duration_sec=duration_sec,
                successful_candidates=0,
                grasp_count=0,
                error_code=MoveItErrorCodes.TIMED_OUT,
            )

        result = action_response.result
        successful_candidate_indexes = {
            ranked_grasp.candidate_index for ranked_grasp in result.ranked_grasps
        }

        return BatchMeasurement(
            candidate_count=candidate_count,
            duration_sec=duration_sec,
            successful_candidates=len(successful_candidate_indexes),
            grasp_count=len(result.ranked_grasps),
            error_code=result.error_code.val,
        )

    def call_service(self, client: Client, request, timeout_sec: float):
        """Call a service and wait for the response with a timeout."""

        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future=future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error(
                f"Service call to {client.srv_name} timed out after {timeout_sec:.1f} seconds."
            )
            return None

        exception = future.exception()
        if exception is not None:
            self.get_logger().error(
                f"Service call to {client.srv_name} raised an exception: {exception}"
            )
            return None

        return future.result()

    def call_action(self, goal: PlanGrasps.Goal, timeout_sec: float):
        """Send an action goal and wait for its final result."""

        goal_future = self.plan_grasp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            goal_future,
            timeout_sec=timeout_sec,
        )

        if not goal_future.done():
            self.get_logger().error(
                f"Sending grasp planning goal timed out after "
                f"{timeout_sec:.1f} seconds."
            )
            return None

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Grasp planning goal was rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=timeout_sec,
        )

        if not result_future.done():
            self.get_logger().error(
                f"Grasp planning action timed out after " f"{timeout_sec:.1f} seconds."
            )
            goal_handle.cancel_goal_async()
            return None

        action_response = result_future.result()
        if action_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Grasp planning action finished with status "
                f"{action_response.status}."
            )

        return action_response

    def run_benchmark(
        self, config: BenchmarkConfig
    ) -> dict[int, list[BatchMeasurement]]:
        """Run the grasp planning benchmark for all candidate counts and repetitions."""

        results: dict[int, list[BatchMeasurement]] = {
            candidate_count: [] for candidate_count in config.candidate_counts
        }

        for candidate_count in config.candidate_counts:
            self.get_logger().info(
                f"Preparing a scene with {candidate_count} candidate objects."
            )

            if not self.remove_existing_benchmark_boxes():
                return {}

            if not self.add_collision_boxes(
                candidate_count,
                timeout_sec=20.0,
            ):
                return {}

            if not self.verify_collision_boxes(candidate_count):
                return {}

            self.get_logger().info(
                "Running one warm-up request before collecting measurements."
            )
            warmup = self.run_action_batch(
                candidate_count=1,
                planning_timeout_sec=config.planning_timeout_sec,
            )

            self.get_logger().info(
                f"Warm-up completed in {warmup.duration_sec:.3f} s "
                f"with {warmup.successful_candidates}/1 successful candidate."
            )

            for repetition in range(1, config.repetitions + 1):
                self.get_logger().info(
                    f"Running candidate_count={candidate_count}, "
                    f"repetition={repetition}/{config.repetitions}."
                )

                measurement: BatchMeasurement = self.run_action_batch(
                    candidate_count=candidate_count,
                    planning_timeout_sec=config.planning_timeout_sec,
                )
                results[candidate_count].append(measurement)

                self.get_logger().info(
                    f"Batch completed in {measurement.duration_sec:.3f} s: "
                    f"{measurement.successful_candidates}/"
                    f"{candidate_count} candidates successful."
                )

        return results


def print_summary(
    results: dict[int, list[BatchMeasurement]],
) -> None:
    """Print aggregate benchmark results."""

    print()
    print("Sequential grasp-planning benchmark")
    print(
        f"{'Candidates':>10} "
        f"{'Runs':>6} "
        f"{'Success':>10} "
        f"{'Mean(s)':>10} "
        f"{'Median(s)':>10} "
        f"{'Min(s)':>10} "
        f"{'Max(s)':>10} "
        f"{'Mean/candidate(s)':>18}"
    )

    for candidate_count, measurements in results.items():
        durations: list[float] = [
            measurement.duration_sec for measurement in measurements
        ]
        successful_candidates: int = sum(
            measurement.successful_candidates for measurement in measurements
        )
        total_candidates: int = candidate_count * len(measurements)
        mean_duration: float = statistics.mean(durations)

        print(
            f"{candidate_count:>10} "
            f"{len(measurements):>6} "
            f"{successful_candidates:>4}/{total_candidates:<5} "
            f"{mean_duration:>10.3f} "
            f"{statistics.median(durations):>10.3f} "
            f"{min(durations):>10.3f} "
            f"{max(durations):>10.3f} "
            f"{mean_duration / candidate_count:>18.3f}"
        )


def parse_arguments() -> BenchmarkConfig:
    parser = argparse.ArgumentParser(
        description=(
            "Measure sequential grasp-planning time as the number of candidate objects increases."
        )
    )
    parser.add_argument(
        "--candidate-counts",
        type=int,
        nargs="+",
        default=DEFAULT_CANDIDATE_COUNTS,
        help="Candidate counts to benchmark. Default: 1 2 4 8 10",
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=DEFAULT_REPETITIONS,
        help="Number of measured runs per candidate count. Default: 5",
    )
    parser.add_argument(
        "--planning-timeout-sec",
        type=float,
        default=10.0,
        help="Total planning timeout for each action request. Default: 10.0 seconds",
    )
    args = parser.parse_args()

    if any(count <= 0 for count in args.candidate_counts):
        parser.error("All candidates count must be greater than zero.")

    if args.repetitions <= 0:
        parser.error("--repetitions must be greater than zero.")

    if args.planning_timeout_sec <= 0:
        parser.error("--planning-timeout-sec must be greater than zero.")

    return BenchmarkConfig(
        candidate_counts=tuple(args.candidate_counts),
        repetitions=args.repetitions,
        planning_timeout_sec=args.planning_timeout_sec,
    )


def main() -> int:
    config = parse_arguments()

    rclpy.init()
    benchmark = GraspPlanningBenchmark()

    try:
        print(f"Benchmark configuration: {config}")

        if not benchmark.wait_for_services():
            return 1

        results: dict[int, list[BatchMeasurement]] = benchmark.run_benchmark(config)
        if not results:
            return 1

        print_summary(results)

        return 0
    finally:
        benchmark.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
