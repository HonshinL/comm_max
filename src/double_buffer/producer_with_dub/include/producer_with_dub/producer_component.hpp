#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "double_buffer_lib/double_buffer.hpp"

using dblbuf::DoubleBuffer;
using dblbuf::DoubleBufferProvider;
using dblbuf::Data;

class ProducerNode : public rclcpp::Node {
public:
  explicit ProducerNode(const rclcpp::NodeOptions & options)
  : Node("producer_node", options)
  {
    buffer_ = DoubleBufferProvider::get();

    // 定时写入 back buffer，并发布一个“tick”消息
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ProducerNode::on_timer, this)
    );

    pub_ = this->create_publisher<std_msgs::msg::String>("tick", 10);
    RCLCPP_INFO(this->get_logger(), "ProducerNode started");
  }

private:
  void on_timer() {
    // 构造示例数据
    Data d;
    d.bytes.resize(5, counter_ % 256);
    buffer_->write(d);

    // 发布 tick 消息
    std_msgs::msg::String msg;
    msg.data = "tick_" + std::to_string(counter_++);
    pub_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(), "Producer wrote back buffer, counter=%ld", counter_);
  }

  std::shared_ptr<DoubleBuffer> buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  long counter_{0};
};