from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='prod_and_coms',
            executable='main_exec',
            name='prod_and_coms_main',
            output='screen'
        )
    ])
