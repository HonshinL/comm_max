#include "rclcpp/rclcpp.hpp"
#include "ringbuffer_lib/ring_buffer.hpp"

class ConsumerNode : public rclcpp::Node {
public:
    ConsumerNode() : Node("consumer_node") {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ConsumerNode::consume, this));
    }

private:
    void consume() {
        auto val = RingBuffer<int>::instance().pop();
        if (val) {
            RCLCPP_INFO(this->get_logger(), "Consumed: %d", *val);
        } else {
            RCLCPP_INFO(this->get_logger(), "Buffer empty");
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
};
