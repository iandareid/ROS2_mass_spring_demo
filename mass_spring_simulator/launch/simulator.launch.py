import os
import sys
import launch.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('mass_spring_simulator'), 'config', 'simulator_config.rviz')

    print(rviz_config_file)

    return LaunchDescription([
        Node(
            package='mass_spring_simulator',
            executable='simulator',
            name='simulator',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])