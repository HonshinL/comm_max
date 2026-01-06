#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <iostream>
#include <atomic>

class RingBuffer {
public:
    explicit RingBuffer(size_t capacity) : buffer_(capacity), capacity_(capacity) {}

    void push(const std::string &data) {
        std::lock_guard<std::mutex> lock(mtx_);
        buffer_[write_index_] = data;
        write_index_ = (write_index_ + 1) % capacity_;
        if (size_ < capacity_) {
            ++size_;
        } else {
            // 覆盖最旧数据
            read_index_ = (read_index_ + 1) % capacity_;
        }
    }

    std::vector<std::string> snapshot() {
        std::lock_guard<std::mutex> lock(mtx_);
        std::vector<std::string> result;
        for (size_t i = 0; i < size_; ++i) {
            size_t idx = (read_index_ + i) % capacity_;
            result.push_back(buffer_[idx]);
        }
        return result;
    }

private:
    std::vector<std::string> buffer_;
    size_t capacity_;
    size_t write_index_{0};
    size_t read_index_{0};
    size_t size_{0};
    std::mutex mtx_;
};

RingBuffer ring_buffer(10); // 容量10
std::atomic<bool> stop_flag{false};

// ROS2 回调：写入环形缓冲区
void callback(const std_msgs::msg::String::SharedPtr msg) {
    ring_buffer.push(msg->data);
    RCLCPP_INFO(rclcpp::get_logger("ringbuffer_demo"), "写入数据: %s", msg->data.c_str());
}

// 消费者线程：周期性读取快照
void consumer_loop(const std::string &name) {
    while (!stop_flag.load()) {
        auto snapshot = ring_buffer.snapshot();
        std::cout << "[" << name << "] 读取快照: ";
        for (auto &d : snapshot) {
            std::cout << d << " ";
        }
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ringbuffer_node");

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
