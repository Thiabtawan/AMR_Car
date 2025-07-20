#!/usr/bin/env python3
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_bot')

    # 1) Declare use_sim_time (false for real robot mapping)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true (real robot => false)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2) Process Xacro to URDF with ros2_control
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={'use_ros2_control': 'true'}
    ).toxml()

    # 3) Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': use_sim_time,
            'transform_tolerance': 0.5,
        }]
    )

    # 4) ros2_control: Controller Manager
    controller_params = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description_xml}, controller_params]
    )
    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    # 5) Controller spawners (diff_drive & joint_state)
    diff_spawner = Node(
        package='controller_manager', executable='spawner.py',
        arguments=['diff_cont'], output='screen'
    )
    joint_spawner = Node(
        package='controller_manager', executable='spawner.py',
        arguments=['joint_broad'], output='screen'
    )
    diff_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager, on_start=[diff_spawner]
        )
    )
    joint_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager, on_start=[joint_spawner]
        )
    )

    # 6) Sensor Nodes: USB Camera & Lidar
    usb_cam_node = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam', output='screen',
        parameters=[
            {'video_device': '/dev/video0'},
            {'image_width': 640}, {'image_height': 480}, {'pixel_format': 'yuyv'}
        ]
    )
    lidar_node = Node(
        package='rplidar_ros', executable='rplidar_composition', name='rplidar', output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 115200,
            'frame_id': 'laser_frame', 'angle_compensate': True, 'scan_mode': 'Standard'
        }]
    )

    # 7) Teleop for manual control
    teleop_node = Node(
        package='teleop_twist_keyboard', executable='teleop_twist_keyboard', name='teleop_keyboard',
        output='screen', prefix='xterm -e',
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
    )

    # 8) SLAM Toolbox for real robot mapping
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    slam_toolbox = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
            {'map_frame': 'map'},
            {'odom_frame': 'odom'},
            {'odom_topic': '/odom'},
            {'subscribe_transforms': False},
            {'publish_map_odom_transform': True},
            {'transform_publish_period': 0.05},  # 20 Hz
            {'do_loop_closing': False},
            {'minimum_laser_range': 0.2},
            {'maximum_laser_range': 12.0},
            {'throttle_scans': 2},
            {'message_filter_queue_size': 100},
            {'tf_buffer_duration': 120.0},
            {'resolution': 0.02},
            {'map_update_interval': 10.0},
        ],
        remappings=[('scan', '/scan')]
    )

    # 9) RViz2 (Fixed Frame = map)
    rviz_config = os.path.join(pkg_share, 'config', 'hw_full.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        emulate_tty=True, arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp_node,
        delayed_controller_manager,
        diff_handler,
        joint_handler,
        usb_cam_node,
        lidar_node,
        teleop_node,
        slam_toolbox,
        rviz_node,
    ])
