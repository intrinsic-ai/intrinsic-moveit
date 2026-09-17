# flowstate_ros_bridge configuration

This document details how to configure the upstream `flowstate_ros_bridge` service (from `sdk-ros`) for use with `intrinsic-moveit`.

By default the `flowstate_ros_bridge` service should already be running on the Intrinsic Core cluster.

---

## Intrinsic Core

In an Intrinsic Core environment (running locally or on edge workcells), the platform services are reachable from the host at `localhost:17080`.

To better understand the configuration changes required, see [this section below](#configuration-requirements).

> [!IMPORTANT]
> **Delete before re-adding**: existing service instances cannot be reconfigured in place under the same name. Any running `flowstate_ros_bridge` instance must be deleted first.

```bash
# Download flowstate_ros_bridge_config.binarypb from a release, https://github.com/intrinsic-ai/intrinsic-moveit/releases

# Delete the running bridge service instance
inctl service delete --address localhost:17080 flowstate_ros_bridge

# Add the service with the new binary configuration
inctl service add --address localhost:17080 ai.intrinsic.flowstate_ros_bridge \
  --config <path-to-config>/flowstate_ros_bridge_config.binarypb
```

If for any reason the configuration needs to be tweaked, check out [the section regarding preparing the `.binarypb`](#compiling-pbtxt-to-binarypb).

---

## Verifying the configuration

Regardless of environment, confirm the bridge is streaming what MoveIt expects:

```bash
# Joint states must be present and carry the six UR joint names
ros2 topic echo /joint_states --once

# The TF tree must contain the robot base frame
ros2 run tf2_ros tf2_echo world ur_module/base_link
```

If `/joint_states` is empty or the joint names do not match the URDF, the bridge is still running with the upstream placeholder configuration — re-check the [configuration requirements](#configuration-requirements).

---

## Configuration requirements

The default upstream `flowstate_ros_bridge` configuration from `sdk-ros` contains generic placeholders (`robot_base_frame_id: "robot/robot/base_link"`, `robot_controller_instance: "robot_controller"`, empty `override_joint_names: []`).

For MoveIt 2 to receive robot joint states and link transforms into the robot kinematic tree, the service must be configured with:

| Field | Required Value | Purpose |
| :--- | :--- | :--- |
| `robot_base_frame_id` | `"ur_module/base_link"` | Anchors the base transform in the TF tree for MoveIt's planning scene. |
| `robot_controller_instance` | `"icon"` | Specifies the active ICON controller instance streaming joint states. |
| `override_joint_names` | `["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]` | Ensures published `/joint_states` match the URDF joint names. |
| `enable_robot_joint_state_topic` | `true` | Enables streaming joint states over `/joint_states`. |
| `enable_force_torque_topic` | `true` | Enables publishing force-torque wrench data. |

These values are applied in both environments; only the delivery mechanism differs. The reference configuration is [`configs/flowstate_ros_bridge_config.pbtxt`](../configs/flowstate_ros_bridge_config.pbtxt).

---

## Building the binary configuration

Applying a configuration with `inctl` requires it as a serialized binary protobuf (`.binarypb`) wrapping a `google.protobuf.Any` message.

**From a local colcon build** — if `flowstate_ros_bridge` and the Intrinsic SDK are built in a ROS 2 workspace, you can find the descriptions in these paths,

```
<path-to-ros2-workspace>/install/intrinsic_sdk_cmake/share/intrinsic_sdk_cmake/intrinsic_proto.desc
<path-to-ros2-workspace>/install/flowstate_ros_bridge/share/flowstate_ros_bridge/flowstate_ros_bridge_protos.desc
```

### Compiling `.pbtxt` to `.binarypb`

The reference configuration is committed at [`configs/flowstate_ros_bridge_config.pbtxt`](../configs/flowstate_ros_bridge_config.pbtxt) and already uses in-cluster DNS addresses.

```bash
cd <path-to-intrinsic-moveit>

protoc --encode=google.protobuf.Any \
  --descriptor_set_in=<path-to-desc>/flowstate_ros_bridge_protos.desc:<path-to-desc>/intrinsic_proto.desc \
  < ./configs/flowstate_ros_bridge_config.pbtxt \
  > ./configs/flowstate_ros_bridge_config.binarypb
```

The resulting `.binarypb` is also gitignored (`configs/*.binarypb`), so regenerate it whenever the `.pbtxt` changes.

---

## Flowstate

Follow official guides to install and add the `flowstate_ros_bridge` service.

Open the solution containing the `flowstate_ros_bridge` service instance, edit that instance's configuration, and set the fields listed in [Configuration requirements](#configuration-requirements).

Use [`configs/flowstate_ros_bridge_config.pbtxt`](../configs/flowstate_ros_bridge_config.pbtxt) as the reference for the exact field names and values to enter.

It is also possible to use `inctl` to apply the service's configurations, however using the platform UI is the most intuitive way.

> [!NOTE]
> Configuration changes take effect when the service instance restarts. After applying them, confirm the bridge is streaming correctly using the checks in [Verifying the configuration](#verifying-the-configuration).
