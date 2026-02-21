"""
Launch: 
ros2 launch pavbot_teensy_control teensy_control.launch.py

ROS commands to check: 
ros2 topic echo /cmd_vel_nav --once
ros2 topic info /teensy/wheels -v

"""

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
            name='cmd_vel_to_wheels',
            output='screen',
            parameters=[params],
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav'),
            ],
        ),
    ])