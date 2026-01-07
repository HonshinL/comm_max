#include "rclcpp/rclcpp.hpp"
#include "ring_buffer_lib/ring_buffer.hpp"

class ProducerNode : public rclcpp::Node {
public:
    ProducerNode() : Node("producer_node") {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ProducerNode::produce, this));
    }

private:
    void produce() {
        static int counter = 0;
        RingBuffer<int>::instance().push(counter);
        RCLCPP_INFO(this->get_logger(), "Produced: %d", counter);
        counter++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
};
