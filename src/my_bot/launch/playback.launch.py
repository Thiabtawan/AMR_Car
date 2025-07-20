#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('my_bot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    default_map = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to map yaml file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock'
    )
    map_arg = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include the navigation launch file from Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'map': map_arg,
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        nav2_launch,
    ])
