from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 RouDi (外部命令)
        ExecuteProcess(
            cmd=['iox-roudi', '-c', '/home/hongxin/Projects/10_Github/cpp_template/comm_max_ws/src/iceoryx/iox_config.toml'],
            output='screen'
        ),

        # 启动 camera publisher 节点
        Node(
            package='camera_pub_pkg',
            executable='camera_pub_with_stamp',
            name='camera_pub',
            output='screen'
        ),

        # 启动 image saver 节点
        Node(
            package='image_saver_pkg',
            executable='image_sub_with_stamp',
            name='image_saver',
            output='screen'
        ),
    ])
