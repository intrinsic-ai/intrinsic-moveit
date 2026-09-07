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

"""Benchmark sequential grasp-planning service requests."""

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

from moveit_planning_interfaces.srv import PlanGrasps

DEFAULT_REQUEST_COUNTS = (1, 2, 4, 8)
DEFAULT_REPETITIONS = 5


@dataclass(frozen=True)
class BenchmarkConfig:
    request_counts: tuple[int, ...]
    repetitions: int
    planning_timeout_sec: float


@dataclass(frozen=True)
class RequestMeasurement:
    object_id: str
    duration_sec: float
    success: bool
    grasp_count: int
    error_code: int


@dataclass(frozen=True)
class BatchMeasurement:
    request_count: int
    duration_sec: float
    request_measurements: tuple[RequestMeasurement, ...]

    @property
    def successful_requests(self) -> int:
        return sum(measurement.success for measurement in self.request_measurements)


class GraspPlanningBenchmark(Node):

    def __init__(self) -> None:
        super().__init__("grasp_planning_benchmark")

        self.apply_planning_scene_client: Client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        self.get_scene_client: Client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

        self.plan_grasp_client: Client = self.create_client(
            PlanGrasps, "/grasp_planning/plan_grasps"
        )

    def wait_for_services(self, timeout_sec: float = 30.0) -> bool:
        """Wait for all required services to be available."""

        clients: tuple[tuple[str, Client], ...] = (
            ("/apply_planning_scene", self.apply_planning_scene_client),
            ("/get_planning_scene", self.get_scene_client),
            ("/grasp_planning/plan_grasps", self.plan_grasp_client),
        )

        for service_name, client in clients:
            self.get_logger().info(f"Waiting for service {service_name}...")

            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(
                    f"Service {service_name} not available after {timeout_sec:.1f} seconds."
                )
                return False
        return True

    def create_collision_box(self) -> CollisionObject:
        """Create the collision box used by all benchmark requests."""

        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "benchmark_box"
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.03, 0.03, 0.03]
        collision_object.primitives.append(primitive)

        pose = Pose()
        pose.position.x = 0.30
        pose.position.y = 0.0
        pose.position.z = 0.20
        pose.orientation.w = 1.0
        collision_object.primitive_poses.append(pose)

        return collision_object

    def add_collision_box(self, timeout_sec: float) -> bool:
        """Add the benchmark box to the planning scene."""

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects.append(self.create_collision_box())

        response = self.call_service(
            self.apply_planning_scene_client, request, timeout_sec=timeout_sec
        )

        if response is None:
            return False

        if not response.success:
            self.get_logger().error("MoveIt rejected the planning scene update.")
            return False

        self.get_logger().info("Added collision box to the planning scene.")

        return True

    def verify_collision_box(self, timeout_sec: float = 10.0) -> bool:
        """Verify that the benchmark box exists in the planning scene."""

        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY

        response = self.call_service(self.get_scene_client, request, timeout_sec)

        if response is None:
            return False

        actual_ids = {
            collision_object.id
            for collision_object in response.scene.world.collision_objects
        }

        if "benchmark_box" not in actual_ids:
            self.get_logger().error("benchmark_box not found in the planning scene.")
            return False

        self.get_logger().info("Verified benchmark_box is in the planning scene.")

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
            if collision_object.id.startswith("benchmark_box")
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

    def create_plan_grasp_request(
        self, planning_timeout_sec: float
    ) -> PlanGrasps.Request:
        """Create a grasp-planning request for the benchmark object."""

        request = PlanGrasps.Request()
        request.group_name = "ur_manipulator"
        request.end_effector_group = "hand"
        request.tool_frame = "hande_tcp"
        request.planning_timeout_sec = planning_timeout_sec
        request.gripper_motion_duration_sec = 0.5
        request.retract_dist_m = 0.1
        request.surfaces = [0, 1, 2, 3, 4, 5]
        request.num_rotations = 4

        request.obj_dims_in_meters.x = 0.03
        request.obj_dims_in_meters.y = 0.03
        request.obj_dims_in_meters.z = 0.03
        request.obj_t_obj_center.orientation.w = 1.0

        request.target.id = "benchmark_box"

        return request

    def plan_request(
        self,
        planning_timeout_sec: float,
    ) -> RequestMeasurement:
        """Execute one grasp-planning request and measure its duration."""

        request = self.create_plan_grasp_request(planning_timeout_sec)

        start_time = time.perf_counter()
        response = self.call_service(
            self.plan_grasp_client, request, timeout_sec=planning_timeout_sec + 5.0
        )
        duration_sec = time.perf_counter() - start_time

        if response is None:
            return RequestMeasurement(
                object_id=request.target.id,
                duration_sec=duration_sec,
                success=False,
                grasp_count=0,
                error_code=MoveItErrorCodes.TIMED_OUT,
            )

        success: bool = response.error_code.val == MoveItErrorCodes.SUCCESS and bool(
            response.grasps
        )

        return RequestMeasurement(
            object_id=request.target.id,
            duration_sec=duration_sec,
            success=success,
            grasp_count=len(response.grasps),
            error_code=response.error_code.val,
        )

    def run_sequential_batch(
        self, request_count: int, planning_timeout_sec: float
    ) -> BatchMeasurement:
        """Execute a batch of requests sequentially and measure the total time."""

        request_measurements: list[RequestMeasurement] = []

        batch_start_time: float = time.perf_counter()

        for request_index in range(request_count):
            measurement = self.plan_request(planning_timeout_sec)
            request_measurements.append(measurement)

            self.get_logger().info(
                f"request={request_index + 1}/{request_count}, "
                f"target={measurement.object_id}: "
                f"{measurement.duration_sec:.3f} s, "
                f"success={measurement.success}, "
                f"grasps={measurement.grasp_count}, "
                f"error_code={measurement.error_code}"
            )

        batch_duration_sec = time.perf_counter() - batch_start_time

        return BatchMeasurement(
            request_count=request_count,
            duration_sec=batch_duration_sec,
            request_measurements=tuple(request_measurements),
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

    def run_benchmark(
        self, config: BenchmarkConfig
    ) -> dict[int, list[BatchMeasurement]]:
        """Run the benchmark for all request counts and repetitions."""

        results: dict[int, list[BatchMeasurement]] = {
            request_count: [] for request_count in config.request_counts
        }

        self.get_logger().info(
            "Running one warm-up request before collecting measurements."
        )
        warmup = self.plan_request(
            planning_timeout_sec=config.planning_timeout_sec,
        )

        self.get_logger().info(
            f"Warm-up completed in {warmup.duration_sec:.3f} s "
            f"with success={warmup.success}."
        )

        for request_count in config.request_counts:
            for repetition in range(1, config.repetitions + 1):
                self.get_logger().info(
                    f"Running request_count={request_count}, "
                    f"repetition={repetition}/{config.repetitions}."
                )

                measurement = self.run_sequential_batch(
                    request_count=request_count,
                    planning_timeout_sec=config.planning_timeout_sec,
                )
                results[request_count].append(measurement)

                self.get_logger().info(
                    f"Batch completed in {measurement.duration_sec:.3f} s: "
                    f"{measurement.successful_requests}/"
                    f"{request_count} requests successful."
                )

        return results


def print_summary(
    results: dict[int, list[BatchMeasurement]],
) -> None:
    """Print aggregate benchmark results."""

    print()
    print("Sequential grasp-planning benchmark")
    print(
        f"{'Requests':>10} "
        f"{'Runs':>6} "
        f"{'Success':>10} "
        f"{'Mean(s)':>10} "
        f"{'Median(s)':>10} "
        f"{'Min(s)':>10} "
        f"{'Max(s)':>10} "
        f"{'Mean/request(s)':>18}"
    )

    for request_count, measurements in results.items():
        durations = [measurement.duration_sec for measurement in measurements]
        successful_requests = sum(
            measurement.successful_requests for measurement in measurements
        )
        total_requests = request_count * len(measurements)
        mean_duration = statistics.mean(durations)

        print(
            f"{request_count:>10} "
            f"{len(measurements):>6} "
            f"{successful_requests:>4}/{total_requests:<5} "
            f"{mean_duration:>10.3f} "
            f"{statistics.median(durations):>10.3f} "
            f"{min(durations):>10.3f} "
            f"{max(durations):>10.3f} "
            f"{mean_duration / request_count:>18.3f}"
        )


def parse_arguments() -> BenchmarkConfig:
    parser = argparse.ArgumentParser(
        description=(
            "Measure how grasp-planning time scales with sequential service requests."
        )
    )
    parser.add_argument(
        "--request-counts",
        type=int,
        nargs="+",
        default=DEFAULT_REQUEST_COUNTS,
        help="Request counts to benchmark. Default: 1 2 4 8",
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=DEFAULT_REPETITIONS,
        help="Number of measured runs per request count. Default: 5",
    )
    parser.add_argument(
        "--planning-timeout-sec",
        type=float,
        default=10.0,
        help="Planning timeout for each request. Default: 10.0 seconds",
    )
    args = parser.parse_args()

    if any(count <= 0 for count in args.request_counts):
        parser.error("All request counts must be greater than zero.")

    if args.repetitions <= 0:
        parser.error("--repetitions must be greater than zero.")

    if args.planning_timeout_sec <= 0:
        parser.error("--planning-timeout-sec must be greater than zero.")

    return BenchmarkConfig(
        request_counts=tuple(args.request_counts),
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

        if not benchmark.remove_existing_benchmark_boxes():
            return 1

        if not benchmark.add_collision_box(
            timeout_sec=20.0,
        ):
            return 1

        if not benchmark.verify_collision_box():
            return 1

        results = benchmark.run_benchmark(config)
        print_summary(results)

        return 0
    finally:
        benchmark.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
