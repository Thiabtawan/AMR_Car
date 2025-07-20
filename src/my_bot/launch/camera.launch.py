from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='cam2image',
            output='screen',
            arguments=['--ros-args', '--remap', 'image:=/camera/image_raw']
        )
    ])
