# Agent Guide: Flowstate MoveIt Service and Skills

This repository provides MoveIt and MoveIt Task Constructor (MTC) planning services and skills tailored for Intrinsic Flowstate solutions. It enables executing geometric motion planning and grasp candidate planning against a consolidated ROS 2 MoveIt planning service using sideloaded C++ skills or standalone ROS 2 interfaces.

## Repository Structure

*   [`moveit_planning_service/`](./moveit_planning_service/): Consolidated ROS 2 planning service handling motion planning (`motion_planning/get_motion_plan`) and grasp planning (`grasp_planning/plan_grasps`) using MoveIt and MoveIt Task Constructor.
*   [`moveit_flowstate_ros_bridge/`](./moveit_flowstate_ros_bridge/): ROS 2 bridge plugin syncing MoveIt planning scenes, TF transforms, collision objects, and robot joint states with Flowstate.
*   [`moveit_planning_interfaces/`](./moveit_planning_interfaces/): ROS 2 service and message definitions (e.g. `PlanGrasps.srv`).
*   [`moveit_plan_motion_skill/`](./moveit_plan_motion_skill/): C++ client skill executing geometric motion planning requests against the planning service.
*   [`moveit_plan_grasp_skill/`](./moveit_plan_grasp_skill/): C++ client skill executing grasp planning requests against the planning service.
*   [`robot_hardware_description/`](./robot_hardware_description/): Hardware description (URDF/Xacro macros and meshes) for the UR5e robot with Robotiq Hand-e gripper, Axia80 FT sensor, and camera mount.
*   [`robot_hardware_moveit_config/`](./robot_hardware_moveit_config/): MoveIt configuration package containing SRDF, kinematics solvers, joint limits, controllers, and planning pipelines (OMPL, STOMP, Pilz).
*   [`Dockerfile.service`](./Dockerfile.service): Multi-stage Dockerfile building and packaging the planning and bridge services into OCI container images.
*   [`Dockerfile.skill`](./Dockerfile.skill): Dockerfile building and packaging the `moveit_plan_motion_skill` and `moveit_plan_grasp_skill` into OCI container images.
*   [`Makefile`](./Makefile): Orchestrates environment setup, image compilation, bundling, and asset installation.
*   [`lyrical.repos`](./lyrical.repos) / [`jazzy.repos`](./jazzy.repos): VCS repository dependency manifests for ROS 2 distributions.

## Quick Start & Sideloading

The sideloading workflow relies on `docker` and Intrinsic CLI tools (`inctl`, `inbuild`).

### Environment Setup

Create a `.env` file based on `.env.template` containing target Intrinsic SDK versions, cluster, and organization info:

```bash
cp .env.template .env
# Edit .env with your environment configuration
```

Download necessary CLI tools:

```bash
make download_inbuild_and_inctl
```

Authenticate with `inctl`:

```bash
export INTRINSIC_ORGANIZATION=...
./bin/inctl auth login --org $INTRINSIC_ORGANIZATION
```

### Build & Deploy Services

> [!IMPORTANT]
> **Service Deployment & Launch Order**: Always build, deploy, and launch `moveit_flowstate_ros_bridge` **before** `moveit_planning_service` when operating in a Flowstate solution.

```bash
# 1. Build & install MoveIt Flowstate ROS Bridge bundle
make install_moveit_flowstate_ros_bridge

# 2. Add Bridge Service to active solution
make add_moveit_flowstate_ros_bridge

# 3. Build & install MoveIt Planning Service bundle
make install_moveit_planning_service

# 4. Add MoveIt Planning Service to active solution
make add_moveit_planning_service
```

### Build & Deploy Planning Skills

```bash
# Build & install the motion planning skill bundle
make install_moveit_plan_motion_skill

# Build & install the grasp planning skill bundle
make install_moveit_plan_grasp_skill
```

## Local Development & Testing in Distrobox

When building and testing locally, use a local ROS 2 environment (e.g. `ubuntu24` distrobox):

```bash
cd <workspace_dir>/src
vcs import . < flowstate_moveit/lyrical.repos
rosdep install --from-paths . --ignore-src -r -y

cd <workspace_dir>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-up-to moveit_flowstate_ros_bridge moveit_planning_service moveit_plan_grasp_skill moveit_plan_motion_skill
colcon test --packages-select moveit_planning_service moveit_plan_grasp_skill moveit_plan_motion_skill
colcon test-result --verbose
```

> [!NOTE]
> **Abseil Dependency Conflicts**: If `sdk-ros/abseil_cpp_vendor` fails to build during local compilation, verify whether system-level `libabsl*` packages are installed on the local system/container. Remove them with `sudo apt remove "libabsl*"` before rebuilding.


### Running Services Locally

#### 1. Flowstate Connected Mode
When connecting to a live Flowstate cluster, start the bridge first:
```bash
# Start Zenoh router tunnel
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:17447"]' ros2 run rmw_zenoh_cpp rmw_zenoh

# Launch bridge
source install/setup.bash
ros2 launch moveit_flowstate_ros_bridge service.launch.py headless:=true start_service_status_monitor:=false

# Launch planning service
ros2 launch moveit_planning_service service.launch.py headless:=false start_service_status_monitor:=false
```

#### 2. Standalone Offline Mode (Without Flowstate)
```bash
source install/setup.bash
ros2 launch moveit_planning_service service.launch.py \
    headless:=false \
    use_mock_hardware:=true \
    expect_collision_objects:=false \
    start_service_status_monitor:=false
```

### Test Service Integration via ROS 2 CLI

```bash
# 1. Apply collision object
ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene "{scene: {is_diff: true, world: {collision_objects: [{header: {frame_id: 'world'}, id: 'test_box', operation: 0, primitives: [{type: 1, dimensions: [0.05, 0.05, 0.05]}], primitive_poses: [{position: {x: 0.3, y: 0.0, z: 0.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}]}}}"

# 2. Plan grasps
ros2 service call /grasp_planning/plan_grasps moveit_planning_interfaces/srv/PlanGrasps "{group_name: 'ur_manipulator', end_effector_group: 'hand', tool_frame: 'hande_tcp', planning_timeout_sec: 5.0, retract_dist_m: 0.1, surfaces: [0, 1, 2, 3, 4, 5], num_rotations: 4, target: {id: 'test_box'}}"

# 3. Plan motion
ros2 service call /motion_planning/get_motion_plan moveit_msgs/srv/GetMotionPlan "{motion_plan_request: {group_name: 'ur_manipulator', num_planning_attempts: 1, allowed_planning_time: 5.0}}"
```

## Technical Notes for Agents

*   **Service Orchestration & Launch Order**:
    *   `moveit_flowstate_ros_bridge` **must be launched before** `moveit_planning_service`.
    *   Upon startup, `moveit_planning_node` performs a retry loop (`trigger_add_collision_objects`) calling `/flowstate_ros_bridge/add_collision_objects` to verify and populate the MoveIt planning scene from Flowstate before declaring itself ready.
    *   The bridge is also responsible for publishing static root/base transforms (`world -> root` and `ur_module/base_link -> base_link`) and `/joint_states` necessary for robot kinematics.
*   **Consolidated Planning Service**:
    *   The `moveit_planning_service` implements a ROS 2 server (`moveit_planning_node`) exposing both motion planning (`motion_planning/get_motion_plan`) and grasp planning (`grasp_planning/plan_grasps`) endpoints.
    *   The service includes full dependencies for `moveit` and `moveit_task_constructor` (`MTC`), enabling both trajectory generation and multi-stage manipulation task execution.
    *   *Implementation status*: Currently, `moveit_planning_node` performs full MTC grasp candidate generation (supporting box annotations, candidate rotation, approach/retreat generation, and pregrasp IK verification). Motion planning endpoint returns responses while full geometric trajectory generators are wired to MoveIt planning pipelines.
*   **Motion Planning Requirements & Geometric Planning Design**:
    *   **MoveIt/MTC Dependencies**: The planning node requires `robot_description` (URDF), `robot_description_semantic` (SRDF), `robot_description_kinematics`, and `planning_pipelines` to be loaded as parameters via `service.launch.py` using `MoveItConfigsBuilder` from `robot_hardware_moveit_config`.
    *   **Launch Architecture & Launch Arguments**: `moveit_planning_service/launch/service.launch.py` orchestrates the complete MoveIt ecosystem, launching `robot_state_publisher`, `move_group` (planning scene & execution action server), `ros2_control_node` with controller spawners, `moveit_planning_node`, `status_monitor`, and RViz2. Key launch arguments include:
        *   `headless` (default: `true`): Controls UI visualization. When `true`, disables RViz (and in the bridge, `rviz_http_proxy`); setting `headless:=false` enables RViz for local visual debugging.
        *   `start_service_status_monitor` (default: `true`): Controls whether the dedicated `status_monitor` node is launched to report operational `ServiceState` to Flowstate. Set to `false` for offline or standalone testing where reporting to Flowstate is not desired.
        *   `use_mock_hardware` (default: `false`): When `true`, starts `ros2_control_node` with controller spawners for offline testing without Flowstate.
        *   `expect_collision_objects` (default: `true`): When `true`, waits for collision objects in the planning scene before marking the planning node ready.
        *   `ros_service_call_timeout_sec` (default: `5.0`): Timeout for internal ROS 2 service calls.
        *   `add_collision_retry_interval_sec` (default: `2.0`): Retry interval for bridge scene population queries.
    *   **Dynamic Planning Scene**:
        *   On each planning request, the planning scene is dynamically built or updated.
        *   **Collision Meshes**: Retrieved from the platform by syncing via `moveit_flowstate_ros_bridge`.
        *   **Transforms**: Retrieved from the `/tf` tree (or a separate transform service).
    *   **Current Joint State**:
        *   Retrieved by listening to and parsing `/joint_states` published by `moveit_flowstate_ros_bridge` or the `joint_state_broadcaster`.
    *   **Geometric Trajectory Planning**:
        *   The motion planning skill focuses on geometric path planning, returning an array of `JointVec` (representing the joint angles of the arm in radians, conforming to `intrinsic_proto.icon.JointVec`).
        *   Velocity and acceleration parameterization is decoupled from the planning phase. A post-processing tool (e.g., TOTG or Ruckig) must be used to calculate point velocities and accelerations based on joint limits.
    *   **Required Motion Planning Inputs**:
        *   `group_name` (string): e.g., `"ur_manipulator"`.
        *   `target_frame` (string): The coordinate frame to align the robot's tool with.
        *   `tool_frame` (string): The robot's tool/end-effector frame (e.g., `hande_tcp`).
        *   `start_joint_state` (`JointVec`, optional): Start configuration (defaults to current joint states retrieved from TF).
        *   `timeout_sec` (double): Maximum allowed planning time.
*   **Grasp Planning Requirements & MTC Integration**:
    *   The `moveit_plan_grasp_skill` interfaces with `grasp_planning/plan_grasps` to generate, evaluate, and select feasible grasp candidates for target objects using MTC task structures.
    *   **Parameter Specification & World Frame Lifecycle**: The skill uses default fallback values (`group_name="ur_manipulator"`, `end_effector_group="hand"`, `timeout_ms=10000.0`, `retract_dist_m=0.1`, `max_num_grasps=1`), scopes target evaluation to `candidate_objects[0]`, and updates pre-allocated output grasp/pregrasp frames in the World Service relative to the candidate object without reparenting.
    *   **Target Object Name Resolution**: The planning node automatically resolves entity hierarchies (e.g. mapping `target_box` or `/target_box` to `target_box/whole` in Flowstate).
*   **Native Protobuf Configuration & Direct Node Loading**:
    *   **Consolidated Binaries**: Both `moveit_flowstate_ros_bridge` and `moveit_planning_service` directly unpack their Protobuf configurations (`MoveItSceneBridgeConfig` and `MoveItPlanningServiceConfig` respectively) from `/etc/intrinsic/runtime_config.pb` inside their main nodes (`moveit_flowstate_ros_bridge_main` and `moveit_planning_node`) and pass overrides via C++ `NodeOptions`.
    *   **No Temporary Files**: Removed external `config_parser` executables and temporary `/tmp/` YAML files across both services, simplifying launch orchestration in `service.launch.py` and `Dockerfile.service`.
    *   **Static Transforms**: `moveit_flowstate_ros_bridge_main` broadcasts static transforms (`world -> root` and `<robot_base_frame_id> -> base_link` with a 180-degree yaw rotation around Z) natively within the main process. Publishing of these static transforms can be toggled via `publish_world_root_tf` and `publish_robot_base_tf` (both default `true`).
*   **Status Monitor & Flowstate `ServiceState` Architecture**:
    *   The `status_monitor` executable in both services implements the `intrinsic_proto::services::v1::ServiceState` gRPC service to report operational health (MoveIt `move_group` `/get_planning_scene`, planning service endpoints, TF tree connectivity, and collision scene population) to the Flowstate Service Manager.
    *   Execution of the `status_monitor` node is toggled via the `start_service_status_monitor` launch argument (defaults to `true`) in both `moveit_planning_service/launch/service.launch.py` and `moveit_flowstate_ros_bridge/launch/service.launch.py`.
*   **License & Copyright Policy**:
    *   The repository is licensed under Apache 2.0.
    *   All source files (`.cpp`, `.h`, `.proto`, `CMakeLists.txt`, `Dockerfile.*`, `Makefile`, etc.) across `flowstate_moveit` must start with the copyright header:
        ```text
        # Copyright 2026 Intrinsic Innovation LLC
        ```
        (or `// Copyright 2026 Intrinsic Innovation LLC` for C++ and Protocol Buffer definitions).
