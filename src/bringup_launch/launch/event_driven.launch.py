from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='ringbuffer_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',  # 多线程容器
            composable_node_descriptions=[
                ComposableNode(
                    package='producer_srv_demo',
                    plugin='ProducerNode',
                    name='producer_node'
                ),
                ComposableNode(
                    package='consumer_srv_demo',
                    plugin='ConsumerNode',
                    name='consumer_node'
                ),
            ],
            output='screen',
        )
    ])
