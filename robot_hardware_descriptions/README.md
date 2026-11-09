# robot_hardware_descriptions

Shared robot hardware descriptions (URDF/Xacro macros and meshes) for `intrinsic-moveit` in ROS 2, supporting multiple workcell configurations side-by-side:

* **`omts`** ([`urdf/omts.urdf.xacro`](./urdf/omts.urdf.xacro)): Vendored UR5e ([`urdf/ur5e/`](./urdf/ur5e/README.md)) + Robotiq Hand-E (`tcp_z_offset="0.14"`) + wrist-mounted Orbbec Gemini 335Le camera.
* **`lab_bb_01`** ([`urdf/lab_bb_01.urdf.xacro`](./urdf/lab_bb_01.urdf.xacro)): Vendored UR3e ([`urdf/ur3e/`](./urdf/ur3e/README.md)) + Robotiq Hand-E (`tcp_z_offset="0.128"`, matching `intrinsic-omts`) + wrist-mounted Orbbec Gemini 335Le camera (`lab_bb_01` calibration).

Third-party Universal Robots UR3e and UR5e visual/collision meshes are provided by [`third_party/robot_meshes_vendor`](../third_party/robot_meshes_vendor/README.md).

> [!NOTE]
> This package is under active development.

## Visualize Robot Descriptions

```bash
# Build the package
colcon build --symlink-install --packages-up-to robot_hardware_descriptions
source install/setup.bash

# Visualize the default OMTS (UR5e) configuration
ros2 launch robot_hardware_descriptions display.launch.py

# Visualize the lab_bb_01 (UR3e) configuration
ros2 launch robot_hardware_descriptions display.launch.py robot_config:=lab_bb_01
```
