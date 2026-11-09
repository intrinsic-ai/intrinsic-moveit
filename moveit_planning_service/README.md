# moveit_planning_service

Consolidated ROS 2 planning and scene synchronization service integrating MoveIt 2, MoveIt Task Constructor (MTC), and in-process World collision scene synchronization (`MoveitSceneSynchronizer`) for geometric motion planning and grasp candidate generation, designed to operate alongside upstream `flowstate_ros_bridge` (which handles `/tf` and `/joint_states`).

Intrinsic Core is the primary supported environment. Licensed users can also deploy the service through Flowstate.

## Launch Arguments Reference (`service.launch.py`)

| Argument | Default | Description |
| :--- | :--- | :--- |
| `robot_config` | `""` (`omts`) | Workcell robot configuration (`omts` for UR5e OMTS, `lab_bb_01` for UR3e `lab_bb_01`). Precedence: CLI argument > `MoveItPlanningServiceConfig.robot_config` in `/etc/intrinsic/runtime_config.pb` > `"omts"` (fails closed if a present, non-empty `runtime_config.pb` cannot be read or decoded). |
| `headless` | `true` | When `true`, omits RViz visualization. When `false`, launches RViz. |
| `start_service_status_monitor` | `true` | Controls whether the `status_monitor` node runs to report `ServiceState` to the platform. |
| `use_mock_hardware` | `false` | When `true`, spawns mock `ros2_control` hardware, virtual joint static TF (`world -> base_link`), and controllers for standalone execution. |
| `intrinsic_core_ingress_address` | `localhost:17080` | Ingress Gateway endpoint for the World service. |
| `zenoh_router_address` | `tcp/localhost:7447` | Address of Zenoh router used for local execution. |
| `publish_world_root_tf` | `true` | In connected mode, controls whether to publish static transform `world -> root`. |
| `publish_robot_base_tf` | `true` | In connected mode, controls whether to publish static transform `robot_base_frame_id -> base_link`. |
| `expect_collision_objects` | `true` | When `true`, blocks planning readiness until collision objects appear in the planning scene. |
| `use_sim_time` | `false` | When `true`, uses simulation clock. |
| `ros_service_call_timeout_sec` | `5.0` | Timeout in seconds for internal ROS 2 service calls. |
| `add_collision_retry_interval_sec` | `2.0` | Interval in seconds between collision scene population checks. |
| `world_sync_interval_sec` | `2.0` | Interval in seconds for periodic background synchronization of World scene objects. |
| `rviz_config` | `...` | Path to custom RViz configuration file. |

> [!NOTE]
> **Parameter Precedence (`robot_config` vs. runtime service parameters)**:
> * **`robot_config` (`CLI > runtime_config.pb > "omts"`)**: Defaults to `""` in `service.launch.py` so an explicit `robot_config:=...` launch argument overrides `/etc/intrinsic/runtime_config.pb` (logging a warning if the two differ) while allowing container deployments with an unset launch argument to read `MoveItPlanningServiceConfig.robot_config`.
> * **Other `MoveItPlanningServiceConfig` parameters (`runtime_config.pb > launch default`)**: Connection and scene-sync launch arguments carry non-empty local development defaults (`localhost:17080`, `tcp/localhost:7447`, `expect_collision_objects:=true`, etc.) so `ros2 launch` works locally when `/etc/intrinsic/runtime_config.pb` is absent. When a valid `MoveItPlanningServiceConfig` is unpacked from `/etc/intrinsic/runtime_config.pb` in a sideloaded container, its values override those local launch defaults.
> * **Fail-closed runtime config loading**: If `/etc/intrinsic/runtime_config.pb` exists and is non-empty, both `service.launch.py` and `moveit_planning_node` fail closed on file I/O or protobuf decode errors (unless `robot_config:=...` was explicitly provided) rather than falling back to `"omts"`.

---

## Integration Testing

Automated launch integration tests (`test_grasp_planning_integration.test.py` for `omts` and `test_grasp_planning_integration_lab_bb_01.test.py` for `lab_bb_01`) verify the grasp planning pipeline across both workcell configurations with mock hardware: starting the planning service, confirming the active URDF/SRDF model, adding multiple collision objects to the planning scene, and sending a multi-candidate goal to the `/grasp_planning/plan_grasps` action.

Run the integration test with colcon:

```bash
colcon test --packages-select moveit_planning_service --event-handlers console_direct+
colcon test-result --verbose
```

---

## Testing ROS 2 Planning Endpoints

After launching the service (see [Building and Running Services Locally](../README.md#3-launching-the-moveit-planning-service-locally)), open a new terminal, source the workspace, and call the service endpoints:

```bash
source install/setup.bash
```

### 1. Add a Collision Object to Planning Scene

```bash
ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene "{
  scene: {
    is_diff: true,
    world: {
      collision_objects: [{
        header: {frame_id: 'world'},
        id: 'test_object',
        operation: 0,
        primitives: [{
          type: 1,
          dimensions: [0.05, 0.05, 0.05]
        }],
        primitive_poses: [{
          position: {x: 0.3, y: 0.0, z: 0.2},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }]
      }]
    }
  }
}"
```

### 2. Request Grasp Planning (`/grasp_planning/plan_grasps`)

The action accepts multiple candidate objects in one goal, divides `planning_timeout_sec` evenly across all candidates, and returns the valid grasps ranked by `grasp_quality`.

```bash
ros2 action send_goal /grasp_planning/plan_grasps \
  moveit_planning_interfaces/action/PlanGrasps \
  "{group_name: 'ur_manipulator',
    end_effector_group: 'hand',
    tool_frame: 'hande_tcp',
    planning_timeout_sec: 5.0,
    gripper_motion_duration_sec: 0.5,
    retract_dist_m: 0.1,
    surfaces: [0, 1, 2, 3, 4, 5],
    num_rotations: 4,
    max_num_grasps: 10,
    candidates: [{
      target: {id: 'test_object'},
      obj_dims_in_meters: {x: 0.05, y: 0.05, z: 0.05},
      obj_t_obj_center: {orientation: {w: 1.0}}
    }]}" \
  --feedback
```

### 3. Request Motion Planning (`/motion_planning/get_motion_plan`)

```bash
ros2 service call /motion_planning/get_motion_plan moveit_msgs/srv/GetMotionPlan "{
  motion_plan_request: {
    group_name: 'ur_manipulator',
    num_planning_attempts: 1,
    allowed_planning_time: 5.0
  }
}"
```

---

## Grasp Planning Benchmark

The benchmark measures the total time required to process an increasing number of candidate objects. Each measured run sends one action goal containing all candidates. The action currently plans those candidates sequentially.

Start the planning service in standalone mode:

```bash
source install/setup.bash
ros2 launch moveit_planning_service service.launch.py \
  robot_config:=omts \
  headless:=true \
  use_mock_hardware:=true \
  expect_collision_objects:=false \
  start_service_status_monitor:=false
```

In another terminal, run:

```bash
source install/setup.bash
ros2 run moveit_planning_service benchmark_grasp_planning.py \
  --candidate-counts 1 2 4 8 10 \
  --repetitions 5 \
  --planning-timeout-sec 10
```

---

## Troubleshooting Guide

### 1. Inspect Service Logs After Startup

Verify that MoveIt and the service endpoints are ready:

```text
[INFO] [moveit_planning_node]: MoveIt Planning Service started (robot_config: 'omts').
[INFO] [moveit_planning_node]: Ready to receive requests at:
[INFO] [moveit_planning_node]:  - motion_planning/get_motion_plan
[INFO] [moveit_planning_node]:  - grasp_planning/plan_grasps (action)
[INFO] [moveit_planning_node]:  - ~/sync_collision_objects
```

### 2. Planning Node Blocking Waiting for Collision Objects

If running without scene objects or bridge, set `expect_collision_objects:=false` and `use_mock_hardware:=true`.

---

## Advertised ROS 2 Interfaces

| Endpoint | Type | Description |
| :--- | :--- | :--- |
| `motion_planning/get_motion_plan` | `moveit_msgs/srv/GetMotionPlan` | Geometric trajectory planning endpoint |
| `grasp_planning/plan_grasps` | `moveit_planning_interfaces/action/PlanGrasps` | Multi-candidate MTC grasp-planning action with ranked results and progress feedback |
| `~/sync_collision_objects` | `std_srvs/srv/Trigger` | In-process World collision scene synchronization endpoint |
| `status_monitor` | `ServiceState` (gRPC) | Platform health and lifecycle service monitor |
