from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 启动你的 ringbuffer_timer 节点
        Node(
            package='ringbuffer_timer',
            executable='main_exec',
            name='ringbuffer_node',
            output='screen'
        ),

        # 启动一个话题发布节点，模拟 ros2 topic pub
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/chatter', 'std_msgs/msg/String', "data:='Hello from ROS2'"],
            output='screen'
        )
    ])
