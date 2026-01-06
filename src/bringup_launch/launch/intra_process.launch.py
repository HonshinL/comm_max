from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='intra_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='pub_demo',
                    plugin='PublisherNode',
                    name='pub_intra_node',
                    extra_arguments=[{'use_intra_process_comms': True}]  # ✅ 节点级别开启
                ),
                ComposableNode(
                    package='sub_demo',
                    plugin='SubscriberNode',
                    name='sub_intra_node',
                    extra_arguments=[{'use_intra_process_comms': True}]  # ✅ 节点级别开启
                ),
            ],
            output='screen',
        )
    ])
