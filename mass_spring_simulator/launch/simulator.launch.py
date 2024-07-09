import os
import sys
import launch.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mass_spring_simulator',
            executable='simulator',
            name='simulator',
            output='screen',
        )
    ])