from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
    producer = ComposableNode(
        package='producer_with_dub',
        plugin='ProducerNode',
        name='producer',
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    consumer = ComposableNode(
        package='consumer_with_dub',
        plugin='ConsumerNode',
        name='consumer',
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    container = ComposableNodeContainer(
        name='double_buffer_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # 多线程容器
        composable_node_descriptions=[producer, consumer],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    return LaunchDescription([container])
