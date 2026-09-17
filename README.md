# Intrinsic MoveIt Planning Service and Skills

![](../media/moveit-plan-grasp-and-move.gif)

This repository provides MoveIt and MoveIt Task Constructor planning services and skills for robotic applications running on [Intrinsic Core](https://github.com/intrinsic-ai/intrinsic-core). It includes a unified ROS 2 planning service (consolidating planning, scene bridging, collision meshes, and TF streaming) and C++ client skills for motion and grasp planning.

Intrinsic Core is the primary supported deployment environment, with the primary supported hardware setup for [Intrinsic Open Machine Tending Solution](https://github.com/intrinsic-ai/intrinsic-omts). Users with a Flowstate license can also deploy and use these components through the Flowstate platform, see [here](docs/flowstate_integration.md).

> [!NOTE]
> This repository is still undergoing active development. Check out [our next steps](#next-steps) for what's to come.

## Repository Structure

*   [`docs/`](./docs/): Configuration and architecture guides:
    *   [`flowstate_integration.md`](./docs/flowstate_integration.md): Instructions on running this integration with Flowstate.
    *   [`flowstate_ros_bridge_configuration.md`](./docs/flowstate_ros_bridge_configuration.md): Required `flowstate_ros_bridge` service configuration values, and how to apply them on Intrinsic Core and Flowstate.
*   [`moveit_planning_service/`](./moveit_planning_service/): Consolidated ROS 2 planning service handling motion planning (`motion_planning/get_motion_plan`), grasp planning (`grasp_planning/plan_grasps`), in-process World scene synchronization, and TF/joint-state streaming.
*   [`moveit_planning_interfaces/`](./moveit_planning_interfaces/): ROS 2 service and message definitions (e.g. `PlanGrasps.srv`).
*   [`moveit_plan_motion_skill/`](./moveit_plan_motion_skill/): C++ client skill executing motion planning requests against the planning service.
*   [`moveit_plan_grasp_skill/`](./moveit_plan_grasp_skill/): C++ client skill executing grasp planning requests against the planning service.
*   [`robot_hardware_description/`](./robot_hardware_description/): Hardware description (URDF/Xacro macros and meshes) for the UR5e robot with Robotiq Hand-E gripper and wrist-mounted Orbbec Gemini 335Le camera.
*   [`robot_hardware_moveit_config/`](./third_party/robot_hardware_moveit_config/): MoveIt configuration package containing SRDF, kinematics solver parameters, joint limits, controllers, and planning pipelines (OMPL, STOMP, Pilz).
*   [`Makefile`](./Makefile): Automation script for building OCI container images, bundling assets and running tests.
*   [`Dockerfile.service`](./Dockerfile.service): Multi-stage Dockerfile building and packaging `moveit_planning_service` into runtime container images.
*   [`Dockerfile.skill`](./Dockerfile.skill): Dockerfile building and packaging skills (`moveit_plan_motion_skill` and `moveit_plan_grasp_skill`) into container images.

## Building and Running Services Locally

You can build and run the services locally in a ROS 2 workspace (e.g., using Ubuntu 26 / Distrobox) for development and testing.

### 1. Set Up Workspace & Import Dependencies

Clone dependencies using the appropriate `.repos` file for your ROS 2 distribution:

```bash
# Create workspace and clone into the source directory
mkdir -p ws_intrinsic_moveit/src
cd ~/ws_intrinsic_moveit/src

git clone https://github.com/intrinsic-ai/intrinsic-moveit.git

# For ROS 2 Lyrical:
vcs import . < intrinsic-moveit/lyrical.repos

# For ROS 2 Jazzy:
# vcs import . < intrinsic-moveit/jazzy.repos

# Install system and ROS dependencies
cd ~/ws_intrinsic_moveit
# Set up rosdep with `sudo rosdep init` and `rosdep update` if not already done
rosdep install --from-paths . --ignore-src -r -y
```

### 2. Build Packages Locally

```bash
cd ~/ws_intrinsic_moveit

# On Lyrical, you might encounter build failures due to mismatch in system library versions, refer to the
# Troubleshooting section for more details.
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-up-to moveit_planning_service moveit_plan_motion_skill moveit_plan_grasp_skill
```

### 3. Launching the MoveIt Planning Service Locally

#### Option A: Intrinsic Core Connected Mode via Zenoh

> [!NOTE]
> **Bridge Requirement**: This integration requires a running `flowstate_ros_bridge` configured for the workcell (to stream `/tf` and `/joint_states`). The bridge should already deployed in the cluster — in which case its configuration must first be updated, see [`docs/flowstate_ros_bridge_configuration.md`](./docs/flowstate_ros_bridge_configuration.md).

If connecting to an active Intrinsic Core cluster, port-forward the Zenoh router port (`7447`):

```bash
# Terminal 1: Port forward zenoh port to localhost
sudo k3s kubectl port-forward -n app-intrinsic-base service/zenoh-router 7447:7447

# Terminal 2: Zenoh router tunnel, these are needed for all ROS interactions with Intrinsic Core
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'

# Launch unified planning service, with visualization using Rviz
source install/setup.bash
ros2 launch moveit_planning_service service.launch.py \
    headless:=false \
    start_service_status_monitor:=false

# Or run in headless mode
# ros2 launch moveit_planning_service service.launch.py \
#     headless:=true \
#     start_service_status_monitor:=false
```

Once launched with rviz, the MoveIt planning scene and robot state should be in sync with Intrinsic Core.

![](../media/moveit-scene-sync.gif)

#### Option B: Standalone / Offline Mode (Mock Hardware)

For local testing without an Intrinsic Core connection or physical hardware, set `use_mock_hardware:=true` and `expect_collision_objects:=false`:

```bash
source install/setup.bash
ros2 launch moveit_planning_service service.launch.py \
    headless:=false \
    use_mock_hardware:=true \
    expect_collision_objects:=false \
    start_service_status_monitor:=false
```

For all planning service launch arguments, refer to [`moveit_planning_service/README.md`](./moveit_planning_service/README.md).

### 4. Testing Integration with ROS 2 Service Calls

Once the service is running, open a new terminal (after sourcing `install/setup.bash`) to test service endpoints:

![](../media/moveit-mock-hardware.gif)

#### Step 1: Add a Collision Object to the Planning Scene

Directly add a test collision box into the MoveIt planning scene:

```bash
ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene "{
  scene: {
    is_diff: true,
    world: {
      collision_objects: [{
        header: {frame_id: 'world'},
        pose: {
          position: {x: 0.0, y: 0.0, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        },
        id: 'target_box',
        operation: 0,
        primitives: [{
          type: 1,
          dimensions: [0.03, 0.03, 0.03]
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

#### Step 2: Test Grasp Planning Service (`/grasp_planning/plan_grasps`)

Send a grasp planning request for the registered `target_box`:

```bash
ros2 service call /grasp_planning/plan_grasps moveit_planning_interfaces/srv/PlanGrasps "{
  group_name: 'ur_manipulator',
  end_effector_group: 'hand',
  tool_frame: 'hande_tcp',
  planning_timeout_sec: 5.0,
  retract_dist_m: 0.1,
  surfaces: [0, 1, 2, 3, 4, 5],
  num_rotations: 4,
  target: {id: 'target_box'}
}"
```

Expected Response:
* `grasps`: Array of generated `moveit_msgs/Grasp` candidates containing joint trajectories for `pre_grasp_posture` and `grasp_posture`.
* `pre_grasp_poses`: Array of `geometry_msgs/PoseStamped` poses for the end-effector.
* `error_code.val`: `1` (`SUCCESS`).

#### Step 3: Test Motion Planning Service (`/motion_planning/get_motion_plan`)

Send a motion planning request to verify the motion planning pipeline:

```bash
ros2 service call /motion_planning/get_motion_plan moveit_msgs/srv/GetMotionPlan "{
  motion_plan_request: {
    group_name: 'ur_manipulator',
    num_planning_attempts: 1,
    allowed_planning_time: 5.0
  }
}"
```

### 5. Code Style & Linting

Verify and reformat code style across all C++ packages using `clang-format`:

```bash
# Check style
cd <workspace_dir>/src/intrinsic-moveit
make format-check

# Format code
cd <workspace_dir>/src/intrinsic-moveit
make format
```

---

## Troubleshooting Guide

### 1. Inspect Service Logs After Startup

Always verify startup logs in the terminal (for local runs) or via the **Text Logs Viewer** in the Intrinsic Solution Dashboard (for sideloaded cluster services):

```text
[INFO] [moveit_planning_node]: Publishing static transform: [world -> root]
[INFO] [moveit_planning_node]: Publishing static transform: [ur_module/base_link -> base_link]
[INFO] [moveit_planning_node]: MoveIt Planning Service started.
[INFO] [moveit_planning_node]: Ready to receive requests at:
[INFO] [moveit_planning_node]:  - motion_planning/get_motion_plan
[INFO] [moveit_planning_node]:  - grasp_planning/plan_grasps
```

### 2. Manual Collision Object Synchronization

To manually trigger collision scene re-synchronization with the World:

```bash
ros2 service call /moveit_planning_node/sync_collision_objects std_srvs/srv/Trigger "{}"
```

### 3. Planning Node Blocking on Startup in Standalone Mode

If running `moveit_planning_service` locally without a connected `flowstate_ros_bridge` or workcell scene, the planning node will block waiting for collision objects by default. Ensure `expect_collision_objects:=false` and `use_mock_hardware:=true` are set:

```bash
ros2 launch moveit_planning_service service.launch.py \
    headless:=false \
    use_mock_hardware:=true \
    expect_collision_objects:=false \
    start_service_status_monitor:=false
```

### 4. Build Failure in `sdk-ros/abseil_cpp_vendor`

When building the workspace locally, if `abseil_cpp_vendor` (from `sdk-ros`) fails with CMake or header conflict errors, check if system Abseil libraries (`libabsl*`) are installed on your host/container. Uninstall them before building again:

```bash
sudo apt remove "libabsl*"
```

### 5. Build failure `error while loading shared libraries: libxml2.so.2: cannot open shared object file: No such file or directory`.

Given that the source build of LLVM within `intrinsic_sdk_cmake` expects the `libxml2` version that was shipped with previous versions of Ubuntu, we add a compatibility symlink for Ubuntu 26's `libxml2.so.2` to the older `libxml2.so.16` with the following command:

```bash
sudo ln -s /usr/lib/x86_64-linux-gnu/libxml2.so.16 /usr/lib/x86_64-linux-gnu/libxml2.so.2
```

---

## Technical Architecture

1. **Unified Service Process**: `moveit_planning_service` hosts both planning engines (`move_group`, `moveit_planning_node`) and in-process World synchronization (`MoveitSceneSynchronizer`), eliminating multi-process IPC and startup race conditions.
2. **In-Process Scene Synchronization**: `MoveitSceneSynchronizer` broadcasts static transforms (`world -> root`, `ur_module/base_link -> base_link`), converting world meshes into MoveIt collision objects, and streaming joint states and `/tf` transforms.
3. **Planning Endpoints**: `moveit_planning_node` advertises ROS 2 services for motion planning (`motion_planning/get_motion_plan`) and grasp planning (`grasp_planning/plan_grasps`), leveraging MoveIt and MoveIt Task Constructor (`MTC`).
4. **Skill Execution**: Sideloaded C++ skills (`moveit_plan_motion_skill` and `moveit_plan_grasp_skill`) run as independent ROS 2 client nodes when triggered by Intrinsic Core, communicating with the planning service over Zenoh/ROS 2 and returning structured protobuf results.

---

## Next Steps

* **Motion planning**: `moveit_plan_motion_skill`, and motion planning capabilities on `moveit_planning_service`.
* **Single source of truth for hardware descriptions**: use upstream URDFs instead of vendoring them in `robot_hardware_description`.
* **Skills to use actions**: interaction between skills and services should be done via ROS actions, instead of services.
* **Pixi workspace migration**: better dependency control and faster build times.
* **Abstract hardware definitions**: to better support different hardware configurations.
