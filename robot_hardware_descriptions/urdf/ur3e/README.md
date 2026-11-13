# Vendored UR3e Robot Description

This directory contains the vendored UR3e robot hardware description derived from `intrinsic-core` (`intrinsic_control/intrinsic/models/robot_definitions/ur/ur3e/ur3e_base.xacro` and `third_party/universal_robots/ur3e/`). Like [`../ur5e/`](../ur5e/README.md), its joint axes and link frames match Intrinsic's runtime UR kinematic conventions rather than the [upstream ROS 2 `ur_description` REP-103 conventions](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy/urdf).

> [!NOTE]
> This vendored version is temporary until a unified robot description source of truth is shared directly between Intrinsic Core and ROS 2 / MoveIt deployments.

---

## Directory Structure

```text
urdf/ur3e/
├── README.md          # Provenance, frame conventions, and joint-limit notes
└── ur3e_macro.xacro   # Main UR3e robot Xacro macro for ROS 2 / MoveIt
```

### Licensing & Provenance

- `ur3e_macro.xacro` is licensed under Apache-2.0 (`Copyright 2026 Intrinsic Innovation LLC`).
- Visual and collision meshes (`meshes/ur3e/visual/` and `meshes/ur3e/collision/`) are packaged separately in [`third_party/robot_meshes_vendor`](../../../third_party/robot_meshes_vendor/README.md) with their upstream Universal Robots / Willow Garage **BSD-3-Clause** `LICENSE` files, as vendored in `intrinsic-core/third_party/universal_robots/ur3e/`.

---

## Integration in `lab_bb_01.urdf.xacro`

`urdf/lab_bb_01.urdf.xacro` incorporates this macro directly:

```xml
<!-- Vendored Intrinsic UR3e macro -->
<xacro:include filename="$(find robot_hardware_descriptions)/urdf/ur3e/ur3e_macro.xacro"/>

<xacro:ur3e_robot
  name="ur3e"
  tf_prefix=""
  parent="">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:ur3e_robot>
```

---

## Technical Nuances: Intrinsic UR3e vs. Upstream ROS `ur_description`

1. **Base Frame Alignment (`ur_module/base_link` -> `base_link`)**:
   - In connected mode, `PublishStaticTransforms` broadcasts `ur_module/base_link -> base_link` with a $180^\circ$ Z rotation (`RotZ(pi)`).
   - `ur3e_macro.xacro` therefore places `shoulder_pan_joint` at `xyz="0 0 0.15185" rpy="0 0 0"` and rotates `base_link` visual/collision meshes by `rpy="0 0 pi"` so every link and mesh aligns 1-to-1 with `ur3e_base.xacro` in World coordinates.
   - As in `ur5e_macro.xacro`, `base_link` uses the non-inflated `collision/base.stl` raised by `0.01 m` along `+Z` to prevent false self-collisions with the mounting surface.
2. **Intrinsic Joint Axes & Kinematics**:
   - `shoulder_pan_joint`: `xyz="0 0 0.15185"`, axis `[0, 0, 1]`, velocity `3.1415927 rad/s`.
   - `shoulder_lift_joint`: `xyz="0 0 0"`, axis `[0, 1, 0]`, bounds `[-4.1, 0.96] rad`, velocity `3.1415927 rad/s`.
   - `elbow_joint`: `xyz="0.24355 0 0"`, axis `[0, 1, 0]`, bounds `[-2.8, 2.8] rad`, velocity `3.1415927 rad/s`.
   - `wrist_1_joint`: `xyz="0.2132 0 0"`, axis `[0, 1, 0]`, velocity `6.2831853 rad/s`.
   - `wrist_2_joint`: `xyz="0 0.13105 -0.08535"`, axis `[0, 0, -1]`, velocity `6.2831853 rad/s`.
   - `wrist_3_joint`: `xyz="0 0.0921 0" rpy="-pi/2 pi 0"`, axis `[0, 0, 1]`, velocity `6.2831853 rad/s`.
   - `flange` and `tool0` are attached to `wrist_3_link` with identity transforms (`rpy="0 0 0"`), matching Intrinsic's tool attachment convention.
3. **Intentional `wrist_3_joint` Position Limit Tightening (`+-2*pi`)**:
   - `intrinsic-core`'s `ur3e_base.xacro` sets `wrist_3_joint` position bounds to `[-1e16, 1e16]` for ICON infinite-turn compatibility.
   - In `ur3e_macro.xacro` (and `lab_bb_01_moveit_config/config/joint_limits.yaml`), `wrist_3_joint` is intentionally bounded to **`[-6.2831853, 6.2831853]` (`+-2*pi` rad)** so MoveIt / OMPL joint-space samplers do not sample astronomical joint values and to protect the tethered wrist-mounted Orbbec Gemini 335Le camera cable.
