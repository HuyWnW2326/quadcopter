import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quadcopter',
            executable='control_position_node',
            output='screen'
        ),
        Node(
            package='quadcopter',
            namespace='quadcopter',
            executable='processes.py',
            name='processes',
            prefix='gnome-terminal --'
        )
    ])