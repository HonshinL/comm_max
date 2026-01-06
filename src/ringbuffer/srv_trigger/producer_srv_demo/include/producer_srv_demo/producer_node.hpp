#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ringbuffer_lib/ring_buffer.hpp"

class ProducerNode : public rclcpp::Node {
public:
    explicit ProducerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("producer_node", options) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("ringbuffer_topic", 10);

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "produce_once",
            std::bind(&ProducerNode::produce, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

private:
    void produce(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        static int counter = 0;
        RingBuffer<int>::instance().push(counter);

        auto msg = std_msgs::msg::Int32();
        msg.data = counter;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Produced: %d", counter);
        counter++;

        response->success = true;
        response->message = "Produced one value";
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};
