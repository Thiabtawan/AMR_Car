import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_bot')
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    # Process XACRO via Python API
    doc = xacro.process_file(xacro_file)
    robot_description_xml = doc.toxml()

    # Robot State Publisher
    # rsp_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': robot_description_xml,
    #         'use_sim_time': False,
    #         'use_ros2_control': True
    #     }]
    # )

    # Controller Manager
    controller_params = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}, controller_params]
    )

    # Delay controller_manager start
    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    # Spawn controllers when manager is up
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['diff_cont'],
        output='screen'
    )
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_broad'],
        output='screen'
    )

    diff_spawner_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )
    joint_spawner_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    return LaunchDescription([
        rsp_node,
        delayed_controller_manager,
        diff_spawner_handler,
        joint_spawner_handler
    ])