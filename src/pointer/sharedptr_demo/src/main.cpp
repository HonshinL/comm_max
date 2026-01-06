#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <iostream>

std::shared_ptr<std::string> shared_data;
std::mutex mtx;
std::atomic<bool> stop_flag{false};

// ROS2 回调：更新共享数据
void callback(const std_msgs::msg::String::SharedPtr msg) {
    auto new_data = std::make_shared<std::string>(msg->data);
    {
        std::lock_guard<std::mutex> lock(mtx);
        shared_data = new_data;
    }
    RCLCPP_INFO(rclcpp::get_logger("sharedptr_demo"), "写入数据: %s", msg->data.c_str());
}

// 消费者线程：周期性读取共享数据
void consumer_loop(const std::string &name) {
    while (!stop_flag.load()) {
        std::shared_ptr<std::string> local_copy;
        {
            std::lock_guard<std::mutex> lock(mtx);
            local_copy = shared_data;
        }
        if (local_copy) {
            std::cout << "[" << name << "] 读取数据: " << *local_copy << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sharedptr_node");

    auto sub = node->create_subscription<std_msgs::msg::String>(
        "chatter", 10, callback);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);

    // 启动多个消费者线程
    std::thread consumer1([] { consumer_loop("Consumer1"); });
    std::thread consumer2([] { consumer_loop("Consumer2"); });

    executor.spin();

    stop_flag.store(true);
    consumer1.join();
    consumer2.join();

    rclcpp::shutdown();
    return 0;
}
