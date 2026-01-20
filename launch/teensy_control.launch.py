from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('pavbot_teensy_control')
    params = os.path.join(pkg, 'config', 'param.yaml')

    return LaunchDescription([
        Node(
            package='pavbot_teensy_control',
            executable='cmd_vel_to_wheels',
            name='cmd_vel_to_axes',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='pavbot_teensy_control',
            executable='teensy_transport_stub',
            name='teensy_transport_stub',
            output='screen',
            parameters=[params],
        ),
    ])
