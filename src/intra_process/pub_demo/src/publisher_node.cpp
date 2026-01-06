#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp_components/register_node_macro.hpp"

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

// 保持 PublisherNode 类定义不变
RCLCPP_COMPONENTS_REGISTER_NODE(PublisherNode)
