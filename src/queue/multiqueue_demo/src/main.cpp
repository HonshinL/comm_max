#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <string>
#include <atomic>
#include <iostream>

struct SharedQueue {
    std::mutex mtx;
    std::condition_variable cv;
    std::queue<std::string> q;
};

SharedQueue queue1;
SharedQueue queue2;
std::atomic<bool> stop_flag{false};

// ROS2 回调：push 数据到多个队列
void callback(const std_msgs::msg::String::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(queue1.mtx);
        queue1.q.push(msg->data);
    }
    queue1.cv.notify_one();

    {
        std::lock_guard<std::mutex> lock(queue2.mtx);
        queue2.q.push(msg->data);
    }
    queue2.cv.notify_one();

    RCLCPP_INFO(rclcpp::get_logger("multiqueue_demo"), "写入数据: %s", msg->data.c_str());
}

// 消费者线程
void consumer_loop(SharedQueue &queue, const std::string &name) {
    while (!stop_flag.load()) {
        std::unique_lock<std::mutex> lock(queue.mtx);
        queue.cv.wait(lock, [&] { return !queue.q.empty() || stop_flag.load(); });

        while (!queue.q.empty()) {
            auto data = queue.q.front();
            queue.q.pop();
            lock.unlock();

            std::cout << "[" << name << "] 消费数据: " << data << std::endl;

            lock.lock();
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("multiqueue_node");

    auto sub = node->create_subscription<std_msgs::msg::String>(
        "chatter", 10, callback);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);

    std::thread consumer1([] { consumer_loop(queue1, "Consumer1"); });
    std::thread consumer2([] { consumer_loop(queue2, "Consumer2"); });

    executor.spin();

    stop_flag.store(true);
    queue1.cv.notify_all();
    queue2.cv.notify_all();
    consumer1.join();
    consumer2.join();

    rclcpp::shutdown();
    return 0;
}
