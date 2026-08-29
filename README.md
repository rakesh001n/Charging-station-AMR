# Charging Station AMR

ROS 2 Humble TurtleBot3 simulation with battery monitoring, charging, and a
visible yellow charging pad.

## Included packages

- `battery_interfaces`: custom battery and robot-state messages.
- `battery_patrol_robot`: Gazebo world, colored TurtleBot3 wrapper, battery
  node, and Day 1/Day 2 launch files.
- `turtlebot3_description`: TurtleBot3 Burger meshes and URDF resources used by
  the simulation.

## Dependencies

Install the ROS 2 Humble dependencies before building:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-xacro ros-humble-teleop-twist-keyboard
```

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch battery_patrol_robot day2_battery.launch.py
```

The battery charges automatically when the robot is within 0.30 m of the
yellow pad at `(0, 0)`. Set `/battery_charging_enable` to `false` to disable
charging. When the battery reaches 0%, the command gate publishes zero
velocity and prevents movement.
