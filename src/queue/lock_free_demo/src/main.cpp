#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/lockfree/queue.hpp>
#include <thread>
#include <atomic>
#include <iostream>

// 无锁队列，容量设为 1024
boost::lockfree::queue<std::string*> shared_queue(1024);
std::atomic<bool> stop_flag{false};

// ROS2 回调：生产者，把消息放入队列
void callback(const std_msgs::msg::String::SharedPtr msg)
{
    auto data = new std::string(msg->data);
    if (!shared_queue.push(data)) {
        std::cerr << "队列已满，丢弃数据: " << msg->data << std::endl;
        delete data;
    }
}

// main_loop：消费者，从队列取数据
void main_loop()
{
    while (!stop_flag.load())
    {
        std::string* data = nullptr;
        if (shared_queue.pop(data)) {
            if (data) {
                std::cout << "main_loop 消费数据: " << *data << std::endl;
                delete data; // 释放内存
            }
        } else {
            // 队列为空时稍作休眠，避免忙等
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("example_node");

    auto sub = node->create_subscription<std_msgs::msg::String>(
        "chatter", 10, callback);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);

    std::thread consumer_thread(main_loop);

    executor.spin(); // 阻塞，处理 ROS2 回调

    stop_flag.store(true);
    consumer_thread.join();

    rclcpp::shutdown();
    return 0;
}
