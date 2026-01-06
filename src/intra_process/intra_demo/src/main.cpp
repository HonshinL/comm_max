#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// Publisher 节点
class PublisherNode : public rclcpp::Node {
public:
    PublisherNode(const rclcpp::NodeOptions & options)
    : Node("publisher_node", options) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("demo_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PublisherNode::publish_msg, this));
    }

private:
    void publish_msg() {
        static int counter = 0;
        auto msg = std::make_unique<std_msgs::msg::Int32>();
        msg->data = counter++;
        RCLCPP_INFO(this->get_logger(), "Publishing: %d", msg->data);
        publisher_->publish(std::move(msg));  // intra-process: 直接传指针
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Subscriber 节点
class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode(const rclcpp::NodeOptions & options)
    : Node("subscriber_node", options) {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "demo_topic",
            10,
            std::bind(&SubscriberNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const std_msgs::msg::Int32::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: %d", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    // ✅ 开启 intra-process 通信
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto publisher = std::make_shared<PublisherNode>(options);
    auto subscriber = std::make_shared<SubscriberNode>(options);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(publisher);
    exec.add_node(subscriber);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
