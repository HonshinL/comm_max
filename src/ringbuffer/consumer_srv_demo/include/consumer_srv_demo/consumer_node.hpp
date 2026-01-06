#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ringbuffer_lib/ring_buffer.hpp"

class ConsumerNode : public rclcpp::Node {
public:
    explicit ConsumerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("consumer_node", options) {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "ringbuffer_topic",
            10,
            std::bind(&ConsumerNode::consume, this, std::placeholders::_1));
    }

private:
    void consume(const std_msgs::msg::Int32::SharedPtr msg) {
        RingBuffer<int>::instance().push(msg->data);  // 可选：也写入本地 RingBuffer
        RCLCPP_INFO(this->get_logger(), "Consumed: %d", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};
