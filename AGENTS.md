# Agent Guide: Intrinsic MoveIt Service and Skills

This repository provides MoveIt and MoveIt Task Constructor (MTC) planning services and skills tailored for Intrinsic Core solutions. It enables executing geometric motion planning and grasp candidate planning against a unified ROS 2 MoveIt planning service using sideloaded C++ skills or standalone ROS 2 interfaces.

Intrinsic Core is the primary supported environment. Licensed users can also deploy these components through Flowstate, the enterprise platform.

## Repository Structure

*   [`docs/`](./docs/): Configuration and deployment guides ([`flowstate_integration.md`](./docs/flowstate_integration.md), [`flowstate_ros_bridge_configuration.md`](./docs/flowstate_ros_bridge_configuration.md)).
*   [`moveit_planning_service/`](./moveit_planning_service/): Consolidated ROS 2 planning service handling motion planning (`motion_planning/get_motion_plan`), grasp planning (`grasp_planning/plan_grasps`), in-process World scene synchronization, and TF/joint-state streaming.
*   [`moveit_planning_interfaces/`](./moveit_planning_interfaces/): ROS 2 service and message definitions (e.g. `PlanGrasps.srv`).
*   [`moveit_plan_motion_skill/`](./moveit_plan_motion_skill/): C++ client skill executing geometric motion planning requests against the planning service.
*   [`moveit_plan_grasp_skill/`](./moveit_plan_grasp_skill/): C++ client skill executing grasp planning requests against the planning service.
*   [`robot_hardware_description/`](./robot_hardware_description/): Hardware description (URDF/Xacro macros and meshes) for the UR5e robot with Robotiq Hand-E gripper and wrist-mounted Orbbec Gemini 335Le camera.
*   [`robot_hardware_moveit_config/`](./third_party/robot_hardware_moveit_config/): MoveIt configuration package containing SRDF, kinematics solvers, joint limits, controllers, and planning pipelines (OMPL, STOMP, Pilz).
*   [`Dockerfile.service`](./Dockerfile.service): Multi-stage Dockerfile building and packaging `moveit_planning_service` into OCI container images.
*   [`Dockerfile.skill`](./Dockerfile.skill): Dockerfile building and packaging the `moveit_plan_motion_skill` and `moveit_plan_grasp_skill` into OCI container images.
*   [`Makefile`](./Makefile): Orchestrates environment setup, image compilation, bundling, and asset installation.
*   [`lyrical.repos`](./lyrical.repos) / [`jazzy.repos`](./jazzy.repos): VCS repository dependency manifests for ROS 2 distributions.

## Flowstate Sideloading

For licensed Flowstate users, the sideloading workflow relies on `docker` and the Intrinsic CLI tools (`inctl`, `inbuild`). See [`docs/flowstate_integration.md`](./docs/flowstate_integration.md) for details.

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

Deploy the unified `moveit_planning_service`:

```bash
# 1. Build & install MoveIt Planning Service bundle
make install_moveit_planning_service

# 2. Add MoveIt Planning Service to active solution
make add_moveit_planning_service
```

Clean up when done:

```bash
make delete_moveit_planning_service
make uninstall_moveit_planning_service
```

### Build & Deploy Planning Skills

```bash
# Build & install the motion planning skill bundle
make install_moveit_plan_motion_skill

# Build & install the grasp planning skill bundle
make install_moveit_plan_grasp_skill
```

## Local Development & Testing in Distrobox

When building and testing locally, use a local ROS 2 environment (e.g. `ubuntu26` distrobox):

```bash
cd <workspace_dir>/src
vcs import . < intrinsic-moveit/lyrical.repos
rosdep install --from-paths . --ignore-src -r -y

cd <workspace_dir>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-up-to moveit_planning_service moveit_plan_grasp_skill moveit_plan_motion_skill
colcon test --packages-select moveit_planning_service moveit_plan_grasp_skill moveit_plan_motion_skill
colcon test-result --verbose

# Code style and linting (clang-format)
# Check style
cd <workspace_dir>/src/intrinsic-moveit
make format-check

# Format code
cd <workspace_dir>/src/intrinsic-moveit
make format
```

> [!NOTE]
> **Abseil Dependency Conflicts**: If `sdk-ros/abseil_cpp_vendor` fails to build during local compilation, verify whether system-level `libabsl*` packages are installed on the local system/container. Remove them with `sudo apt remove "libabsl*"` before rebuilding.


### Running Services Locally

#### Option A: Intrinsic Core Connected Mode via Zenoh
When connecting to an active Intrinsic Core or Flowstate cluster:
```bash
# Terminal 1: Port-forward Zenoh router port (7447)
sudo k3s kubectl port-forward -n app-intrinsic-base service/zenoh-router 7447:7447

# Terminal 2: Zenoh router tunnel configuration for ROS interactions
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'

# Launch upstream flowstate_ros_bridge (streams /tf and /joint_states)
# Note: If flowstate_ros_bridge is already running as a cluster service, ensure its configuration
# has been updated with configs/flowstate_ros_bridge_config.pbtxt before running MoveIt.
source install/setup.bash
ros2 launch moveit_planning_service flowstate_ros_bridge.launch.py

# Terminal 3: Launch MoveIt planning service (MoveIt core, collision scene, planning endpoints)
source install/setup.bash
ros2 launch moveit_planning_service service.launch.py headless:=false start_service_status_monitor:=false
```

#### Option B: Standalone / Offline Mode (Mock Hardware)
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

*   **Unified Service Architecture & Decoupled Upstream Bridge**:
    *   `moveit_planning_service` handles planning engines (`move_group`, `moveit_planning_node`), in-process World collision scene synchronization (`MoveitSceneSynchronizer`), and lifecycle health monitoring (`status_monitor`).
    *   Upstream `flowstate_ros_bridge` handles general ROS 2 bridging: streaming `/tf`, `/tf_sim`, `/joint_states`, and FTS wrench from Zenoh.
    *   **Upstream Bridge Configuration Requirement**: The default `sdk-ros` bridge configuration ships generic placeholders (`robot_base_frame_id: "robot/robot/base_link"`, `robot_controller_instance: "robot_controller"`, empty `override_joint_names: []`). Left unchanged, MoveIt receives no usable joint states and cannot resolve kinematics. Any bridge instance used with `intrinsic-moveit` must therefore match [`configs/flowstate_ros_bridge_config.pbtxt`](./configs/flowstate_ros_bridge_config.pbtxt) — chiefly `robot_base_frame_id: "ur_module/base_link"`, `robot_controller_instance: "icon"`, and the six UR joint names. When launched locally via `flowstate_ros_bridge.launch.py`, these are already applied as launch argument defaults. For instances deployed in Intrinsic Core or Flowstate, see [`docs/flowstate_ros_bridge_configuration.md`](./docs/flowstate_ros_bridge_configuration.md) for the full field list and how to apply it.
    *   Upon startup, `moveit_planning_node` performs in-process scene verification, broadcasts static root/base transforms (`world -> root` and `ur_module/base_link -> base_link`), and verifies that collision objects and `/joint_states` are populated before unblocking planning requests.
*   **Consolidated Planning Service**:
    *   The `moveit_planning_service` implements a ROS 2 server (`moveit_planning_node`) exposing both motion planning (`motion_planning/get_motion_plan`) and grasp planning (`grasp_planning/plan_grasps`) endpoints.
    *   The service includes full dependencies for `moveit` and `moveit_task_constructor` (`MTC`), enabling both trajectory generation and multi-stage manipulation task execution.
    *   *Implementation status*: Currently, `moveit_planning_node` performs full MTC grasp candidate generation (supporting box annotations, candidate rotation, approach/retreat generation, and pregrasp IK verification). Motion planning endpoint returns responses while full geometric trajectory generators are wired to MoveIt planning pipelines.
*   **Motion Planning Requirements & Geometric Planning Design**:
    *   **MoveIt/MTC Dependencies**: The planning node requires `robot_description` (URDF), `robot_description_semantic` (SRDF), `robot_description_kinematics`, and `planning_pipelines` to be loaded as parameters via `service.launch.py` using `MoveItConfigsBuilder` from `robot_hardware_moveit_config`.
    *   **Launch Architecture & Launch Arguments**: `moveit_planning_service/launch/service.launch.py` orchestrates the complete MoveIt ecosystem, launching `robot_state_publisher`, `move_group` (planning scene & execution action server), `ros2_control_node` with controller spawners, `moveit_planning_node`, `status_monitor`, and RViz2. Key launch arguments include:
        *   `headless` (default: `true`): Controls UI visualization. When `true`, disables RViz; setting `headless:=false` enables RViz for local visual debugging.
        *   `start_service_status_monitor` (default: `true`): Controls whether the dedicated `status_monitor` node is launched to report operational `ServiceState` to the platform. Set to `false` for offline or standalone testing where reporting to the platform is not desired.
        *   `use_mock_hardware` (default: `false`): When `true`, starts `ros2_control_node` with controller spawners for offline testing without platform connection.
        *   `intrinsic_core_ingress_address` (default: `localhost:17080`): Ingress Gateway gRPC endpoint for the World service.
        *   `zenoh_router_address` (default: `tcp/localhost:7447`): Address of Zenoh router for local execution.
        *   `publish_world_root_tf` (default: `true`): In connected mode, controls whether to publish static transform `world -> root`.
        *   `publish_robot_base_tf` (default: `true`): In connected mode, controls whether to publish static transform `robot_base_frame_id -> base_link`.
        *   `strip_tf_prefixes` (default: `[""]`): List of TF / object name prefixes to strip when synchronizing World scene objects.
        *   `expect_collision_objects` (default: `true`): When `true`, waits for collision objects in the planning scene before marking the planning node ready.
        *   `use_sim_time` (default: `false`): Uses simulation clock.
        *   `ros_service_call_timeout_sec` (default: `5.0`): Timeout for internal ROS 2 service calls.
        *   `add_collision_retry_interval_sec` (default: `2.0`): Retry interval for bridge scene population queries.
        *   `world_sync_interval_sec` (default: `2.0`): Interval in seconds for periodic background synchronization of World scene objects.
    *   **Dynamic Planning Scene**:
        *   On each planning request, the planning scene is dynamically built or updated.
        *   **Collision Geometries**: Retrieved from the World service via `MoveitSceneSynchronizer` (supporting both glTF meshes and primitive bounding shapes like boxes, cylinders, and spheres).
        *   **Collision Filtering**: `MoveitSceneSynchronizer` filters out robot hardware entities (`excluded_collision_namespaces`: `"ur_module"`, `"gripper"`, `"camera_mount"`, `"orbbec_camera"`, `"ecat_ft_hal_module_with_adapters_with_sim"`) to prevent hardware links already defined in the robot URDF from being duplicated as external collision obstacles in the planning scene.
        *   **Transforms**: Retrieved from the `/tf` tree.
    *   **Current Joint State**:
        *   Retrieved by listening to and parsing `/joint_states` published by `flowstate_ros_bridge` (in connected mode) or `joint_state_broadcaster` (in mock hardware mode).
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
    *   **Fail-Fast Skill Execution**: If grasp planning fails to generate reachable/collision-free grasp candidates or the MoveIt planning service returns an error code, `moveit_plan_grasp_skill` returns a non-OK `absl::Status` (e.g. `absl::NotFoundError`, `absl::InvalidArgumentError`, `absl::FailedPreconditionError`) to fail the skill execution immediately and prevent downstream nodes from executing on uninitialized grasp poses.
    *   **Robot Model Caching**: `GraspPlanningPipeline` caches the `moveit::core::RobotModel` (via `robot_model_loader::RobotModelLoader`) across planning requests, setting the cached model into `mtc::Task` via `task.setRobotModel(...)` to eliminate redundant URDF/SRDF XML parsing, parameter lookups, and kinematics solver re-initialization.
    *   **Parameter Specification & World Frame Lifecycle**: The skill uses default fallback values (`group_name="ur_manipulator"`, `end_effector_group="hand"`, `timeout_ms=15000.0`, `retract_dist_m=0.05`, `max_num_grasps=1`), iterates across all `candidate_objects` to generate and rank grasps by quality, and updates pre-allocated output grasp/pregrasp frames in the World Service relative to the best-ranked candidate object without reparenting.
    *   **Target Object Name Resolution**: The planning node automatically resolves entity hierarchies (e.g. mapping `target_box` or `/target_box` to `target_box/whole` in the World service).
*   **Native Protobuf Configuration & Typed Struct Architecture (Approach B)**:
    *   **Decoupled Components**: Configuration is resolved cleanly in `main()` from `/etc/intrinsic/runtime_config.pb` or launch parameters and passed to `PublishStaticTransforms` (`StaticTransformsConfig`) and `MoveitSceneSynchronizer` (`MoveitSceneSynchronizerConfig`) as strongly typed C++ structs.
    *   **Lifecycle Node Ready**: Component classes do not query ROS 2 parameters directly, eliminating get/set race conditions and facilitating future migration to `rclcpp_lifecycle::LifecycleNode`.
    *   **Static Transforms**: `PublishStaticTransforms` broadcasts static transforms (`world -> root` and `<robot_base_frame_id> -> base_link` with a 180-degree yaw rotation around Z) natively within the main process in connected mode (`use_mock_hardware:=false`). In mock hardware mode (`use_mock_hardware:=true`), `static_virtual_joint_tfs.launch.py` handles the MoveIt virtual joint `world -> base_link`. Publishing of static transforms can be toggled via `publish_world_root_tf` and `publish_robot_base_tf` (both default `true`).
*   **Status Monitor & ServiceState Architecture**:
    *   The `status_monitor` executable in `moveit_planning_service` implements the `intrinsic_proto::services::v1::ServiceState` gRPC service to report operational health (MoveIt `move_group` `/get_planning_scene`, planning service endpoints, `/joint_states` freshness and arm joint presence, MoveIt planning scene `robot_state`, TF chain connectivity `world -> base_link -> tool_frame`, and collision scene population) to the platform Service Manager.
    *   Execution of the `status_monitor` node is toggled via the `start_service_status_monitor` launch argument (defaults to `true`) in `moveit_planning_service/launch/service.launch.py`.
*   **Code Style & Linting Guidelines**:
    *   This repository follows the default formatting style enforced by `clang-format`.
    *   **Always check code style before committing changes**:
        ```bash
        cd <workspace_dir>/src/intrinsic-moveit
        make format-check
        ```
    *   **Automatically fix style discrepancies**:
        ```bash
        cd <workspace_dir>/src/intrinsic-moveit
        make format
        ```
    *   CI runs style checks on pull requests via GitHub Actions. Always verify that modified C++ files are cleanly formatted and pass `ament_uncrustify`.
*   **License & Copyright Policy**:
    *   The repository is licensed under Apache 2.0.
    *   All source files (`.cpp`, `.h`, `.proto`, `CMakeLists.txt`, `Dockerfile.*`, `Makefile`, etc.) across `intrinsic-moveit` must start with the copyright header:
        ```text
        # Copyright 2026 Intrinsic Innovation LLC
        ```
        (or `// Copyright 2026 Intrinsic Innovation LLC` for C++ and Protocol Buffer definitions).
