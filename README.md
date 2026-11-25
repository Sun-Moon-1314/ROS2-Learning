# ROS2-Learning

Personal ROS 2 learning workspace with C++ and Python examples, robot descriptions, control and hardware experiments, Gazebo simulation, and MoveIt configurations.

## Environment

- ROS 2 Humble
- Ubuntu 22.04
- `colcon` and `rosdep`

Some packages use additional ROS 2 components such as MoveIt, Gazebo, `ros2_control`, and the Dynamixel SDK. Install dependencies required by the packages you intend to build.

## Packages

The `src/` directory contains 15 packages:

- Basic nodes and actions: `my_cpp_pkg`, `my_py_pkg`, `actions_cpp`, `actions_py`, `my_robot_interfaces`, `turtlesim_catch_them_all`
- Robot models and launch files: `my_robot_description`, `my_robot_bringup`
- Controllers and hardware: `my_robot_controllers`, `my_robot_hardware`, `dxl_test`
- Motion planning and commanders: `my_robot_moveit_config`, `my_combined_robot_moveit_config`, `my_robot_commander_cpp`, `my_robot_commander_py`

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

To build a single package, use `colcon build --packages-select <package_name>`.

The Gazebo world at `src/my_robot_bringup/worlds/test_world.sdf` references models and meshes at local absolute paths. Update those paths or provide the assets locally before using that world on another computer.
