# moveit_flowstate_ros_bridge

## MoveitSceneBridge

A plugin for streaming collision meshes to the MoveIt planning scene, as well as publishing TF transformations and robot joint states (`/joint_states`) directly from Flowstate (similar to `WorldBridge` in `sdk-ros`).

> [!IMPORTANT]
> **Do NOT run `MoveitSceneBridge` and `WorldBridge` together.** Running both bridges concurrently will cause conflicting TF frame and `/joint_states` broadcasts. (Note: Future refactoring will decouple joint state and TF streaming into independent modular bridge components).

## Quickstart

### Building

```bash
cd ~/ws_flowstate_moveit/src
vcs import . < flowstate_moveit/lyrical.repos
# use jazzy.repos if using ROS 2 Jazzy

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-up-to moveit_flowstate_ros_bridge
```

### Running Locally

Running locally against an Intrinsic Flowstate cluster requires port forwarding the cluster's Zenoh router port `17447` to `localhost:17447`:

```bash
# 1. Start Zenoh router pointing to localhost:17447
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:17447"]' ros2 run rmw_zenoh_cpp rmw_zenoh

# 2. In another terminal, launch the bridge service
source install/setup.bash
ros2 launch moveit_flowstate_ros_bridge service.launch.py headless:=true start_service_status_monitor:=false
```

### Launch Arguments Reference

| Argument | Default | Description |
| :--- | :--- | :--- |
| `headless` | `true` | When `true`, disables `rviz_http_proxy`. When `false`, starts `rviz_http_proxy` on port 8123. |
| `start_service_status_monitor` | `true` | Controls whether the `status_monitor` node runs to report `ServiceState` to Flowstate. |
| `use_sim_time` | `false` | When `true`, uses ROS 2 simulation clock. Note that if running as a Flowstate service, `flowstate_ros_gz_bridge` needs to be sideloaded too. |

### Sideloading into Flowstate

From the repository root (`flowstate_moveit`):

```bash
# Build & install Bridge Service bundle
make install_moveit_flowstate_ros_bridge

# Add Bridge Service to active solution
make add_moveit_flowstate_ros_bridge
```

---

## Troubleshooting Guide

### 1. Inspect Service Logs After Startup

Check terminal output or the **Text Logs Viewer** in the Intrinsic Solution Dashboard to verify that static transforms are active and the bridge plugin has started:

```text
Publishing static transform: [world -> root]
Publishing static transform: [ur_module/base_link -> base_link]
```

### 2. Missing Collision Objects in Planning Scene

MoveIt's planning scene monitor subscribes to `/collision_object`. If `moveit_planning_service` started after the bridge or missed the initial message, trigger synchronization manually:

```bash
ros2 service call /flowstate_ros_bridge/add_collision_objects std_srvs/srv/Trigger "{}"
```

---

## ROS 2 Interfaces

### Published Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `tf` | `tf2_msgs/msg/TFMessage` | Transformations of objects from Flowstate |
| `/joint_states` | `sensor_msgs/msg/JointState` | Robot joint positions, velocities, and efforts |
| `workcell_markers/visual` | `visualization_msgs/msg/MarkerArray` | Visual meshes for SceneObjects |
| `workcell_markers/collision` | `visualization_msgs/msg/MarkerArray` | Collision meshes for SceneObjects |
| `/collision_object` | `moveit_msgs/msg/CollisionObject` | Collision objects published directly to the MoveIt planning scene |

### Services

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `flowstate_get_resource` | `flowstate_interfaces/srv/GetResource` | Provides binary GLTF resources for scene meshes |
| `/flowstate_ros_bridge/add_collision_objects` | `std_srvs/srv/Trigger` | Triggers republishing and syncing collision objects into the MoveIt planning scene |
