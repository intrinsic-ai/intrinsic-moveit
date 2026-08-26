# Vendored UR5e Robot Description

This directory contains the vendored UR5e robot hardware description, derived from Intrinsic's internal robot setup. It contains joint, frame, and link parameterizations that differ from the [upstream open-source description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy/urdf).

> [!NOTE]
> This vendored version is temporary until a single source of truth is established where the deployed robot description is derived directly from an upstream or unified package. Once available, this vendored directory will be refactored or removed.

---

## Directory Structure

```text
urdf/ur5e/
├── README.md                                    # This documentation
├── ur5e_macro.xacro                             # Main UR5e robot Xacro macro for ROS 2 / MoveIt
├── ur5e.urdf                                    # Intrinsic-derived raw URDF model
├── visual/                                      # Intrinsic visual DAE meshes
├── collision/                                   # Intrinsic collision STL meshes
└── model.config                                 # Model configuration manifest
```

---

## Integration in `robot.urdf.xacro`

The top-level `robot.urdf.xacro` incorporates this vendored macro directly:

```xml
<!-- Vendored Intrinsic UR5e macro -->
<xacro:include filename="$(find robot_hardware_description)/urdf/ur5e/ur5e_macro.xacro"/>

<xacro:ur5e_robot
  name="ur5e"
  tf_prefix=""
  parent="world">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:ur5e_robot>
```

---

## Technical Nuances & Differences: Intrinsic vs. Upstream ROS URDF (REP-103)

When performing motion planning or grasp candidate planning in MoveIt against a physical robot running Intrinsic's runtime, it is essential to understand the frame, joint, link, and angle parameterization differences between Intrinsic's setup and the upstream open-source ROS `ur_description` URDF setup (which adheres strictly to **ROS REP-103** coordinate system conventions).

### 1. Frame Tree & Link Hierarchy Comparison

| Feature / Frame | Upstream ROS URDF (`ur_description`) | Intrinsic Format (`ur5e_macro.xacro`) | MoveIt / Grasp Planning Impact |
| :--- | :--- | :--- | :--- |
| **Root Link** | `base_link` with child `base_link_inertia` | `base_link` (no separate inertia link) | In upstream ROS URDF, inertia is placed on `base_link_inertia` for KDL solver rules. Intrinsic places inertia directly on `base_link`. |
| **UR Controller Base** | `base` (rotated `rpy="0 0 pi"` relative to `base_link`) | Not present (`base_link` used directly) | In upstream ROS, `base` aligns with the Teach Pendant's `/Base` frame. |
| **Coordinate Conventions** | **REP-103 Compliant**: Standardized Z-up/X-forward orientations and tool flange conventions. | **Local CAD Alignment**: Direct link frame alignments without REP-103 frame offsets. | Frames target local CAD link axes. Grasp pose poses must account for identity flange alignment. |
| **End-Effector Flange** | `flange` (rotated `rpy="0 -pi/2 -pi/2"` relative to `wrist_3_link` per REP-103) | `flange` (attached to `wrist_3_link` with identity transform `rpy="0 0 0"`) | **CRITICAL:** Upstream `flange` is rotated by $-90^\circ$ around Y and Z per REP-103. Intrinsic `flange` is 1-to-1 identical with `wrist_3_link`. |
| **Standard Tool Frame** | `tool0` (rotated `rpy="pi/2 0 pi/2"` relative to `flange` to restore REP-103 orientation) | `tool0` (attached with identity transform `rpy="0 0 0"` to `flange`) | In upstream ROS URDF, `tool0` Z-axis points outward per REP-103. In Intrinsic's, `tool0` and `flange` are both identity-aligned with `wrist_3_link`. |

---

### 2. Joint Axes and Local Frame Parameterization

The upstream open-source ROS URDF follows **Denavit-Hartenberg (DH)** parameter conventions where all joint axes point along their local **Z-axis** (`xyz="0 0 1"`). In contrast, Intrinsic uses direct local frame alignment where joint rotation axes are aligned to local **Y-axes** (`xyz="0 1 0"`) or negative Z-axes.

| Joint Name | Upstream ROS URDF Origin & Axis | Intrinsic URDF Origin & Axis | Frame / Angle Nuance |
| :--- | :--- | :--- | :--- |
| **`shoulder_pan_joint`** | Origin: `xyz="0 0 0.1625" rpy="0 0 0"`<br>Axis: `xyz="0 0 1"` | Origin: `xyz="0 0 0.1625" rpy="0 0 0"`<br>Axis: `xyz="0 0 1"` | Physically identical net transform. |
| **`shoulder_lift_joint`** | Origin: `xyz="0 0 0" rpy="1.570796327 0 0"`<br>Axis: `xyz="0 0 1"` | Origin: `xyz="0 0 0" rpy="0 0 0"`<br>Axis: `xyz="0 1 0"` | **Axis Mismatch:** Upstream rotates joint 2 around Z-axis after a $90^\circ$ roll. Intrinsic rotates directly around local Y-axis `[0, 1, 0]`. |
| **`elbow_joint`** | Origin: `xyz="-0.425 0 0" rpy="0 0 0"`<br>Axis: `xyz="0 0 1"` | Origin: `xyz="0.425 0 0" rpy="0 0 0"`<br>Axis: `xyz="0 1 0"` | **Sign Mismatch:** Upstream translation is `x = -0.425` due to DH frame rotation. Intrinsic translation is `x = +0.425` along local arm extension. Axis is `[0, 1, 0]`. |
| **`wrist_1_joint`** | Origin: `xyz="-0.3922 0 0.1333" rpy="0 0 0"`<br>Axis: `xyz="0 0 1"` | Origin: `xyz="0.3922 0 0" rpy="0 0 0"`<br>Axis: `xyz="0 1 0"` | **Translation Offset:** Upstream includes `z = 0.1333` offset at `wrist_1`. Intrinsic places `y = 0.1333` offset at `wrist_2_joint`. |
| **`wrist_2_joint`** | Origin: `xyz="0 -0.0997 0" rpy="1.570796327 0 0"`<br>Axis: `xyz="0 0 1"` | Origin: `xyz="0 0.1333 -0.0997" rpy="0 0 0"`<br>Axis: `xyz="0 0 -1"` | **Axis Sign Inversion:** Upstream axis is `+Z`. Intrinsic axis is **`-Z` (`xyz="0 0 -1"`)**. |
| **`wrist_3_joint`** | Origin: `xyz="0 0.0996 0" rpy="1.570796326 3.14159 3.14159"`<br>Axis: `xyz="0 0 1"` | Origin: `xyz="0 0.0996 0" rpy="-1.570796 0 0"`<br>Axis: `xyz="0 0 1"` | Roll is $-90^\circ$ in Intrinsic vs $+90^\circ$ in upstream ROS URDF. |

---

### 3. Summary for Grasp Planning & MoveIt Integration

When executing motion planning or defining grasp planning skills in MoveIt:
1. **1-to-1 Joint Reflection**: Because `ur5e_macro.xacro` incorporates Intrinsic's exact URDF joint axes (including `wrist_2_joint` axis `[0, 0, -1]`), joint state vectors published from the physical robot or Intrinsic runtime reflect 1-to-1 in MoveIt without requiring manual joint angle conversions.
2. **Target Frame Selection**: `flange` and `tool0` are both provided with identity transforms to `wrist_3_link`, matching Intrinsic's tool attachment conventions.
3. **MoveIt Compatibility**: PickIK / KDL numerical solvers in MoveIt parse this URDF tree natively.
