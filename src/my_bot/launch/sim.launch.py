#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paths to package shares
    pkg_share = get_package_share_directory('my_bot')
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')

    # 1) Declare simulation time
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2) Launch Gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'pause': 'false',
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 3) Prepare robot_description from Xacro without ros2_control plugin
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=false use_sim_time:=', use_sim_time
        ]),
        value_type=str
    )

    # 4) Robot State Publisher node
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )

    # 5) Spawn the robot into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot'
        ],
        output='screen'
    )

    # 6) Teleop after a delay
    teleop = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_keyboard',
                output='screen',
                prefix='xterm -e',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    # 7) SLAM Toolbox after Gazebo is up
    slam_toolbox = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    # 8) RViz after SLAM starts
    rviz_config = os.path.join(pkg_share, 'config', 'hw_full.rviz')
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-d', rviz_config]
            )
        ]
    )

    # 9) Assemble launch description
    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_state_pub,
        spawn_robot,
        teleop,
        slam_toolbox,
        rviz,
    ])
