# Charging Station AMR

This project is a ROS 2 Humble simulation of a battery-aware autonomous
mobile robot based on the TurtleBot3 Burger. The robot patrols a Gazebo world,
avoids obstacles, returns to a visible yellow charging pad when its battery is
low, and resumes its mission after charging.

## What Was Implemented

- Gazebo TurtleBot3 simulation with local robot resources.
- Blue TurtleBot3 chassis, orange lidar, and black wheels.
- Yellow visual charging pad at `(0, 0)`.
- Battery drain from travelled distance and idle time.
- Automatic charging inside the docking radius.
- Zero-battery command lock.
- Odometry-based waypoint patrol.
- Laser-based obstacle avoidance.
- A* grid path planning.
- Alternate waypoint selection when a route is blocked.
- Low-battery return to the charging pad.
- Automatic patrol recovery after charging.
- Emergency stop, mission pause, and mission resume controls.
- Battery reset service for testing.
- Diagnostics and mission telemetry.
- RViz robot, laser scan, TF, and planned-path visualization.

## Progress By Day

- **Day 1:** Gazebo world, TurtleBot3 model, teleoperation, RViz, odometry, TF,
  and lidar.
- **Day 2:** Battery messages, battery drain, charging logic, charging pad,
  and zero-battery movement protection.
- **Day 3:** Autonomous waypoint patrol and robot state reporting.
- **Day 4:** A* navigation, obstacle avoidance, alternate routes, low-battery
  return, and emergency stop.
- **Day 5:** Diagnostics, telemetry, battery reset service, and RViz planned
  path display.
- **Day 6:** Operator pause and resume services.
- **Day 7:** Automated validation tests, final launch, and documentation.

## Packages

- `battery_interfaces`: custom `BatteryState`, `RobotState`, and
  `ResetBattery` interfaces.
- `battery_patrol_robot`: simulation, battery node, navigation, diagnostics,
  launch files, and RViz configuration.
- `turtlebot3_description`: TurtleBot3 meshes and URDF resources used by the
  simulation.

## Build

Install the ROS dependencies:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-xacro ros-humble-teleop-twist-keyboard
```

Build the project:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Files

```bash
ros2 launch battery_patrol_robot simulation.launch.py
ros2 launch battery_patrol_robot battery_simulation.launch.py
ros2 launch battery_patrol_robot patrol.launch.py
ros2 launch battery_patrol_robot navigation.launch.py
ros2 launch battery_patrol_robot monitoring.launch.py
ros2 launch battery_patrol_robot operations.launch.py
ros2 launch battery_patrol_robot final_system.launch.py
```

## Topics

- `/odom`: robot odometry.
- `/scan`: lidar scan.
- `/raw_cmd_vel`: commands from patrol/navigation.
- `/cmd_vel`: battery-gated commands consumed by Gazebo.
- `/battery_state`: battery percentage and charging state.
- `/robot_state`: current mission state and waypoint.
- `/planned_path`: current A* path for RViz.
- `/diagnostics`: standard diagnostic status.
- `/system_diagnostics`: readable telemetry summary.
- `/emergency_stop`: emergency-stop Boolean command.

## Services

Reset the battery:

```bash
ros2 service call /reset_battery battery_interfaces/srv/ResetBattery "{level: 50.0}"
```

Pause and resume the mission:

```bash
ros2 service call /pause_mission std_srvs/srv/Trigger "{}"
ros2 service call /resume_mission std_srvs/srv/Trigger "{}"
```

## Behavior

The A* planner uses the known obstacle geometry in the simulated world and
inflates obstacles for the robot footprint. Laser readings provide immediate
local avoidance. If a waypoint is blocked, the planner selects a reachable
alternate waypoint. If the battery falls to `25%`, the robot plans a route to
the charging pad. It stops there and charges until `80%`, then resumes patrol.
At `0%`, the battery node publishes zero velocity regardless of incoming
commands.

## Validation

Run the automated checks:

```bash
python3 -m unittest discover -s src/battery_patrol_robot/test -p 'test_*.py'
```

See [`docs/day7_validation.md`](docs/day7_validation.md) for the complete
manual test checklist and report screenshot suggestions.
