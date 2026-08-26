# moveit_planning_service

Consolidated ROS 2 planning service integrating MoveIt 2 and MoveIt Task Constructor (MTC) for geometric motion planning and grasp candidate generation.

> [!IMPORTANT]
> **Launch Order**: When running alongside Flowstate, launch **`moveit_flowstate_ros_bridge` before `moveit_planning_service`**. `moveit_planning_service` queries `/flowstate_ros_bridge/add_collision_objects` and expects transforms and joint states from the bridge before marking itself ready.

## Local Building and Running (Offline Mode)

### 1. Build

Build the planning service alongside the robot configuration and interface definitions:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-up-to moveit_planning_service
```

### 2. Launch

Source the workspace setup script:

```bash
source install/setup.bash
```

#### Visual Mode (RViz + Mock Hardware)

Runs MoveIt with RViz visualization, controller spawners, and mock hardware for offline testing without Flowstate:

```bash
ros2 launch moveit_planning_service service.launch.py \
    headless:=false \
    use_mock_hardware:=true \
    expect_collision_objects:=false \
    start_service_status_monitor:=false
```

#### Headless Mode

Runs the planning node and MoveIt background servers without RViz:

```bash
ros2 launch moveit_planning_service service.launch.py \
    headless:=true \
    use_mock_hardware:=true \
    expect_collision_objects:=false \
    start_service_status_monitor:=false
```

### Launch Arguments Reference

| Argument | Default | Description |
| :--- | :--- | :--- |
| `headless` | `true` | When `true`, omits RViz. When `false`, launches RViz visualization. |
| `start_service_status_monitor` | `true` | Controls whether the `status_monitor` node runs to report `ServiceState` to Flowstate. |
| `use_mock_hardware` | `false` | When `true`, spawns mock `ros2_control` hardware and controllers for standalone execution. |
| `expect_collision_objects` | `true` | When `true`, blocks planning readiness until collision objects appear in the planning scene. |
| `ros_service_call_timeout_sec` | `5.0` | Timeout in seconds for internal ROS 2 service calls. |
| `add_collision_retry_interval_sec` | `2.0` | Interval in seconds between collision scene population checks. |
| `rviz_config` | `...` | Path to custom RViz configuration file. |

---

## Integration Testing

An automated launch integration test (`test_grasp_planning_integration.test.py`) verifies the entire pipeline with mock hardware: starting the planning service, adding a collision object to the planning scene, verifying synchronization, and calling `/grasp_planning/plan_grasps`.

Run the test with colcon:

```bash
colcon test --merge-install --packages-select moveit_planning_service --event-handlers console_direct+
colcon test-result --verbose
```

---

## Testing ROS 2 Services

Open a new terminal, source the workspace, and call the service endpoints:

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

If running without Flowstate scene objects or bridge, set `expect_collision_objects:=false` and `use_mock_hardware:=true`.

---

## Advertised ROS 2 Interfaces

| Service Name | Type | Description |
| :--- | :--- | :--- |
| `motion_planning/get_motion_plan` | `moveit_msgs/srv/GetMotionPlan` | Geometric trajectory planning endpoint |
| `grasp_planning/plan_grasps` | `moveit_planning_interfaces/srv/PlanGrasps` | MTC-based box candidate grasp generator |
| `status_monitor` | `ServiceState` (gRPC) | Flowstate health and lifecycle service monitor |
