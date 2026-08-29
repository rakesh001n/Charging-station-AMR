import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('battery_patrol_robot')
    turtlebot_share = get_package_share_directory('turtlebot3_description')

    world = LaunchConfiguration('world')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    rviz = LaunchConfiguration('rviz')
    teleop = LaunchConfiguration('teleop')
    cmd_vel_input = LaunchConfiguration('cmd_vel_input')

    robot_description_file = os.path.join(
        package_share, 'urdf', 'turtlebot3_burger_gazebo.urdf.xacro')
    robot_description = xacro.process_file(
        robot_description_file,
        mappings={'namespace': ''},
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )),
        launch_arguments={
            'world': world,
            'gui': gazebo_gui,
            'pause': 'false',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
        ],
        output='screen',
    )

    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        remappings=[('cmd_vel', cmd_vel_input)],
        condition=IfCondition(teleop),
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_share, 'rviz', 'day1.rviz')],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        # Gazebo Classic's Qt client is more reliable through XWayland on this
        # system, and this world does not need the remote model database.
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.pathsep.join([
                os.path.dirname(turtlebot_share),
                '/usr/share/gazebo-11/models',
                os.environ.get('GAZEBO_MODEL_PATH', ''),
            ]),
        ),
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            os.pathsep.join([
                os.path.dirname(turtlebot_share),
                '/usr/share/gazebo-11',
                os.environ.get('GAZEBO_RESOURCE_PATH', ''),
            ]),
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(package_share, 'worlds', 'patrol_world.world'),
            description='Gazebo world file',
        ),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'teleop',
            default_value='true',
            description='Start keyboard teleoperation in this terminal',
        ),
        DeclareLaunchArgument(
            'cmd_vel_input',
            default_value='/cmd_vel',
            description='Topic used as the teleoperation command input',
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        keyboard_teleop,
        rviz_node,
    ])
