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
            os.path.join(package_share, 'launch', 'simulation.launch.py')),
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
        parameters=[{'use_sim_time': True, 'charging_enabled': True}],
        output='screen',
    )
    navigation = Node(
        package='battery_patrol_robot',
        executable='navigation_node',
        name='navigation_node',
        parameters=[{
            'use_sim_time': True,
            'obstacle_inflation': 0.25,
            'waypoint_tolerance': 0.08,
            'max_linear_speed': 0.35,
        }],
        output='screen',
    )
    return LaunchDescription([simulation, battery, navigation])
