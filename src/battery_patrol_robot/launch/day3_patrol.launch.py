import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('battery_patrol_robot')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'day1_sim.launch.py')),
        launch_arguments={
            'teleop': 'false',
            'rviz': 'true',
            'cmd_vel_input': '/raw_cmd_vel',
        }.items(),
    )
    battery = Node(
        package='battery_patrol_robot',
        executable='battery_node',
        name='battery_node',
        parameters=[{
            'use_sim_time': True,
            'charging_enabled': True,
            'charger_x': 0.0,
            'charger_y': 0.0,
            'docking_distance': 0.30,
        }],
        output='screen',
    )
    patrol = Node(
        package='battery_patrol_robot',
        executable='patrol_node',
        name='patrol_node',
        parameters=[{
            'use_sim_time': True,
            'low_battery_level': 25.0,
            'resume_battery_level': 80.0,
            'charger_x': 0.0,
            'charger_y': 0.0,
            'docking_distance': 0.30,
        }],
        output='screen',
    )
    return LaunchDescription([simulation, battery, patrol])
