import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('battery_patrol_robot')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'navigation.launch.py')),
    )
    diagnostics = Node(
        package='battery_patrol_robot',
        executable='diagnostics_node',
        name='diagnostics_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([navigation, diagnostics])
