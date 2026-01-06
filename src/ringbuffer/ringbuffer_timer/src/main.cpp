#include "rclcpp/rclcpp.hpp"
#include "ringbuffer_timer/producer_node.hpp"
#include "ringbuffer_timer/consumer_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto producer = std::make_shared<ProducerNode>();
    auto consumer = std::make_shared<ConsumerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(producer);
    executor.add_node(consumer);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
