#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <string>

std::mutex mtx;
std::condition_variable cv;
std::queue<std::string> shared_queue;
bool stop_flag = false;

// ROS2 回调：作为生产者，把消息放入队列
void callback(const std_msgs::msg::String::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(mtx);
        shared_queue.push(msg->data);
    }
    cv.notify_one(); // 通知消费者线程
}

// main_loop：作为消费者，从队列取数据
void main_loop()
{
    while (!stop_flag)
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [] { return !shared_queue.empty() || stop_flag; });

        while (!shared_queue.empty())
        {
            auto data = shared_queue.front();
            shared_queue.pop();
            lock.unlock();

            // 处理数据
            std::cout << "main_loop 消费数据: " << data << std::endl;

            lock.lock();
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

    {
        std::lock_guard<std::mutex> lock(mtx);
        stop_flag = true;
    }
    cv.notify_all();
    consumer_thread.join();

    rclcpp::shutdown();
    return 0;
}
