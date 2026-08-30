# Day 7 Validation

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select battery_interfaces battery_patrol_robot
source install/setup.bash
```

Run the automated checks:

```bash
python3 -m unittest discover -s src/battery_patrol_robot/test -p 'test_*.py'
```

## Final System

```bash
ros2 launch battery_patrol_robot final_system.launch.py
```

For a headless validation run:

```bash
ros2 launch battery_patrol_robot final_system.launch.py gazebo_gui:=false rviz:=false
```

## Manual Checks

- Confirm the robot patrols between waypoints.
- Confirm the robot turns around obstacles using `/scan`.
- Confirm `/planned_path` changes when a waypoint is blocked.
- Reset the battery and verify low-battery return:

```bash
ros2 service call /reset_battery battery_interfaces/srv/ResetBattery "{level: 20.0}"
```

- Confirm the robot stops on the yellow pad and publishes `is_charging: true`.
- Confirm emergency stop:

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```

- Confirm mission pause and resume:

```bash
ros2 service call /pause_mission std_srvs/srv/Trigger "{}"
ros2 service call /resume_mission std_srvs/srv/Trigger "{}"
```

- Confirm diagnostics:

```bash
ros2 topic echo /system_diagnostics
```
