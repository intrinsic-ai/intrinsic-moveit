# moveit_planning_service

Consolidated ROS 2 planning and scene synchronization service integrating MoveIt 2, MoveIt Task Constructor (MTC), and in-process World collision scene synchronization (`MoveitSceneSynchronizer`) for geometric motion planning and grasp candidate generation, designed to operate alongside upstream `flowstate_ros_bridge` (which handles `/tf` and `/joint_states`).

Intrinsic Core is the primary supported environment. Licensed users can also deploy the service through Flowstate.

## Launch Arguments Reference (`service.launch.py`)

| Argument | Default | Description |
| :--- | :--- | :--- |
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

---

## Integration Testing

An automated launch integration test (`test_grasp_planning_integration.test.py`) verifies the grasp planning pipeline with mock hardware: starting the planning service, adding a collision object to the planning scene, verifying synchronization, and calling `/grasp_planning/plan_grasps`.

Run the integration test with colcon:

```bash
colcon test --packages-select moveit_planning_service --event-handlers console_direct+
colcon test-result --verbose
```

---

## Testing ROS 2 Services

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

```bash
ros2 service call /grasp_planning/plan_grasps moveit_planning_interfaces/srv/PlanGrasps "{
  group_name: 'ur_manipulator',
  end_effector_group: 'hand',
  tool_frame: 'hande_tcp',
  planning_timeout_sec: 5.0,
  retract_dist_m: 0.1,
  surfaces: [0, 1, 2, 3, 4, 5],
  num_rotations: 4,
  target: {id: 'test_object'}
}"
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

## Troubleshooting Guide

### 1. Inspect Service Logs After Startup

Verify that MoveIt and the service endpoints are ready:

```text
[INFO] [moveit_planning_node]: MoveIt Planning Service started.
[INFO] [moveit_planning_node]: Ready to receive requests at:
[INFO] [moveit_planning_node]:  - motion_planning/get_motion_plan
[INFO] [moveit_planning_node]:  - grasp_planning/plan_grasps
```

### 2. Planning Node Blocking Waiting for Collision Objects

If running without scene objects or bridge, set `expect_collision_objects:=false` and `use_mock_hardware:=true`.

---

## Advertised ROS 2 Interfaces

| Service Name | Type | Description |
| :--- | :--- | :--- |
| `motion_planning/get_motion_plan` | `moveit_msgs/srv/GetMotionPlan` | Geometric trajectory planning endpoint |
| `grasp_planning/plan_grasps` | `moveit_planning_interfaces/srv/PlanGrasps` | MTC-based box candidate grasp generator |
| `~/sync_collision_objects` | `std_srvs/srv/Trigger` | In-process World collision scene synchronization endpoint |
| `status_monitor` | `ServiceState` (gRPC) | Platform health and lifecycle service monitor |
