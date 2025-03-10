import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'quadcopter'

    return LaunchDescription([
        Node(
            package='quadcopter',
            executable='control_position_node',
            name='control_position_node',
            output='screen'
        ),
        Node(
            package='quadcopter',
            executable='processes',
            name='processes',
            prefix='terminator -x bash -c "terminator -u -x ros2 run px4_offboard processes"'
        )
    ])