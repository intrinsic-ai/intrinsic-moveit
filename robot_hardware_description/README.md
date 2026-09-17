# robot_hardware_description

This package contains the robot hardware description for `intrinsic-moveit` in ROS 2.

> [!NOTE]
> This package is under active development.

# Visualize robot description

```bash
# Install dependencies via rosdep, then build the package
colcon build --symlink-install --packages-up-to robot_hardware_description

# Run the basic display demo
source install/setup.bash
ros2 launch robot_hardware_description display.launch.py
```
