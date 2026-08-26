# Flowstate MoveIt Planning Service and Skills

This repository provides MoveIt and MoveIt Task Constructor planning services and skills tailored for Intrinsic Flowstate solutions. It includes a consolidated ROS 2 planning service along with C++ client skills for executing motion planning and grasp planning requests against Flowstate workcells and standalone environments.

## Repository Structure

*   [`moveit_planning_service/`](./moveit_planning_service/): Consolidated ROS 2 planning service handling motion planning (`motion_planning/get_motion_plan`) and grasp planning (`grasp_planning/plan_grasps`) using MoveIt and MoveIt Task Constructor (MTC).
*   [`moveit_flowstate_ros_bridge/`](./moveit_flowstate_ros_bridge/): ROS 2 bridge plugin syncing MoveIt planning scenes, TF transforms, collision meshes, and robot joint states with Flowstate.
*   [`moveit_planning_interfaces/`](./moveit_planning_interfaces/): ROS 2 service and message definitions (e.g. `PlanGrasps.srv`).
*   [`moveit_plan_motion_skill/`](./moveit_plan_motion_skill/): C++ client skill executing motion planning requests against the planning service.
*   [`moveit_plan_grasp_skill/`](./moveit_plan_grasp_skill/): C++ client skill executing grasp planning requests against the planning service.
*   [`robot_hardware_description/`](./robot_hardware_description/): Hardware description (URDF/Xacro macros and meshes) for the UR5e robot with Robotiq Hand-e gripper, Axia80 FT sensor, and camera mount.
*   [`robot_hardware_moveit_config/`](./robot_hardware_moveit_config/): MoveIt configuration package containing SRDF, kinematics solver parameters, joint limits, controllers, and planning pipelines (OMPL, STOMP, Pilz).
*   [`Makefile`](./Makefile): Automation script for building OCI container images, bundling assets, installing to Intrinsic clusters, and running tests.
*   [`Dockerfile.service`](./Dockerfile.service): Multi-stage Dockerfile building and packaging `moveit_planning_service` or `moveit_flowstate_ros_bridge` into lightweight runtime container images.
*   [`Dockerfile.skill`](./Dockerfile.skill): Dockerfile building and packaging sideloaded skills (`moveit_plan_motion_skill` and `moveit_plan_grasp_skill`) into container images.

## Building and Running Services Locally

You can build and run the services locally in a ROS 2 workspace (e.g., using Ubuntu 24 / Distrobox) for development and testing.

> [!IMPORTANT]
> **Service Launch Order**: When operating with a Flowstate cluster, launch **`moveit_flowstate_ros_bridge` before `moveit_planning_service`**. The `moveit_planning_service` verifies the planning scene at startup by calling `/flowstate_ros_bridge/add_collision_objects` and requires static transforms (`world -> root`, `ur_module/base_link -> base_link`) and `/joint_states` published by the bridge.

### 1. Set Up Workspace & Import Dependencies

Clone dependencies using the appropriate `.repos` file for your ROS 2 distribution:

```bash
cd ~/ws_flowstate_moveit/src

# For ROS 2 Lyrical:
vcs import . < flowstate_moveit/lyrical.repos

# For ROS 2 Jazzy:
# vcs import . < flowstate_moveit/jazzy.repos

# Install system and ROS dependencies
rosdep install --from-paths . --ignore-src -r -y
```

### 2. Build Packages Locally

```bash
cd ~/ws_flowstate_moveit
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-up-to moveit_flowstate_ros_bridge moveit_planning_service moveit_plan_motion_skill moveit_plan_grasp_skill
```

### 3. Launching the Flowstate ROS Bridge Locally (Flowstate Connected)

If connecting to an active Flowstate cluster, port-forward the Zenoh router port (`17447`) and launch the bridge first:

```bash
# Terminal 1: Zenoh router tunnel
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:17447"]' ros2 run rmw_zenoh_cpp rmw_zenoh

# Terminal 2: Launch bridge service
source install/setup.bash
ros2 launch moveit_flowstate_ros_bridge service.launch.py headless:=true start_service_status_monitor:=false
```

For all bridge launch arguments, refer to [`moveit_flowstate_ros_bridge/README.md`](./moveit_flowstate_ros_bridge/README.md).

### 4. Launching the MoveIt Planning Service Locally

In another terminal, source the workspace and launch the planning service:

```bash
source install/setup.bash
```

#### Option A: Connected Mode (Flowstate Bridge Active)

```bash
# Visual mode with RViz
ros2 launch moveit_planning_service service.launch.py \
    headless:=false \
    start_service_status_monitor:=false

# Headless mode
ros2 launch moveit_planning_service service.launch.py \
    headless:=true \
    start_service_status_monitor:=false
```

#### Option B: Standalone / Offline Mode (No Flowstate Bridge)

For offline testing without Flowstate or physical hardware, set `use_mock_hardware:=true` and `expect_collision_objects:=false`:

```bash
ros2 launch moveit_planning_service service.launch.py \
    headless:=false \
    use_mock_hardware:=true \
    expect_collision_objects:=false \
    start_service_status_monitor:=false
```

For all planning service launch arguments, refer to [`moveit_planning_service/README.md`](./moveit_planning_service/README.md).

### 5. Testing Integration with ROS 2 Service Calls

Once services are running, open a new terminal (after sourcing `install/setup.bash`) to test service endpoints:

#### Step 1: Add a Collision Object to the Planning Scene

Directly add a test collision box into the MoveIt planning scene:

```bash
ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene "{
  scene: {
    is_diff: true,
    world: {
      collision_objects: [{
        header: {frame_id: 'world'},
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

---

## Build & Deploy to Flowstate Cluster (Sideloading)

For deploying services and skills directly to an Intrinsic Flowstate cluster:

### Environment Setup

Create a `.env` file based on `.env.template` containing target Intrinsic SDK versions, cluster, and organization info:

```bash
cp .env.template .env
# Edit .env with your environment configuration (SDK_VERSION, INTRINSIC_ORGANIZATION, INTRINSIC_CLUSTER, ROS_DISTRO)
```

Download required build tools (`inctl` and `inbuild`):

```bash
make download_inbuild_and_inctl
```

Authenticate with `inctl`:

```bash
export INTRINSIC_ORGANIZATION=...
./bin/inctl auth login --org $INTRINSIC_ORGANIZATION
```

### Build & Deploy Services

Build and deploy `moveit_flowstate_ros_bridge` first, followed by `moveit_planning_service`:

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

Clean up when done:

```bash
make delete_moveit_planning_service
make uninstall_moveit_planning_service
make delete_moveit_flowstate_ros_bridge
make uninstall_moveit_flowstate_ros_bridge
```

### Build & Deploy Planning Skills

```bash
# Build & install the motion planning skill bundle
make install_moveit_plan_motion_skill

# Build & install the grasp planning skill bundle
make install_moveit_plan_grasp_skill
```

Uninstall skills when done:

```bash
make uninstall_moveit_plan_motion_skill
make uninstall_moveit_plan_grasp_skill
```

---

## Troubleshooting Guide

### 1. Inspect Service Logs After Startup

Always verify startup logs in the terminal (for local runs) or via the **Text Logs Viewer** in the Intrinsic Solution Dashboard (for sideloaded cluster services):

* **`moveit_flowstate_ros_bridge`**:
  Confirm that static transforms are broadcasted and plugins are loaded:
  ```text
  Publishing static transform: [world -> root]
  Publishing static transform: [ur_module/base_link -> base_link]
  ```

* **`moveit_planning_service`**:
  Confirm that MoveIt and the service endpoints have successfully started:
  ```text
  [INFO] [moveit_planning_node]: MoveIt Planning Service started.
  [INFO] [moveit_planning_node]: Ready to receive requests at:
  [INFO] [moveit_planning_node]:  - motion_planning/get_motion_plan
  [INFO] [moveit_planning_node]:  - grasp_planning/plan_grasps
  ```

### 2. Missing Collision Objects in Planning Scene (Late-Joining Subscriber)

MoveIt's planning scene monitor subscribes to the `/collision_object` topic. If `moveit_planning_service` started after the bridge or was a late-joining subscriber that missed the initial broadcast, manually trigger scene object synchronization from Flowstate:

```bash
ros2 service call /flowstate_ros_bridge/add_collision_objects std_srvs/srv/Trigger "{}"
```

### 3. Planning Node Blocking on Startup in Standalone Mode

If running `moveit_planning_service` locally without a Flowstate bridge or workcell scene, the planning node will block waiting for collision objects by default. Ensure `expect_collision_objects:=false` and `use_mock_hardware:=true` are set:

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

---

## Technical Architecture

1. **Bridge Launch**: `moveit_flowstate_ros_bridge` starts first, synchronizing Flowstate scene objects as MoveIt collision meshes, streaming `/tf` transforms and `/joint_states`, and publishing static base and world transforms.
2. **Service Launch**: `moveit_planning_service` launches `service.launch.py`, starting `robot_state_publisher`, `move_group`, and `moveit_planning_node`. It triggers collision object population via `/flowstate_ros_bridge/add_collision_objects`.
3. **Service Endpoints**: `moveit_planning_node` advertises ROS 2 services for motion planning (`motion_planning/get_motion_plan`) and grasp planning (`grasp_planning/plan_grasps`), leveraging MoveIt and MoveIt Task Constructor (`MTC`).
4. **Skill Execution**: Sideloaded C++ skills (`moveit_plan_motion_skill` and `moveit_plan_grasp_skill`) run as independent ROS 2 client nodes when triggered by Flowstate, communicating with the planning service over Zenoh/ROS 2 and returning structured protobuf results.
