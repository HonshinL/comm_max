#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp_components/register_node_macro.hpp"

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

// ✅ 注册为组件，容器才能加载
RCLCPP_COMPONENTS_REGISTER_NODE(SubscriberNode)
