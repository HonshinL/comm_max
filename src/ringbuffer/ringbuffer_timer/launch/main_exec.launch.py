from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ringbuffer_timer',
            executable='main_exec',
            name='ringbuffer_timer_main',
            output='screen'
        )
    ])
