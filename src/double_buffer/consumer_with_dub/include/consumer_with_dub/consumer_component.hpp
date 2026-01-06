#include "rclcpp/rclcpp.hpp"
#include "double_buffer_lib/double_buffer.hpp"
#include "std_msgs/msg/string.hpp"

using dblbuf::DoubleBuffer;
using dblbuf::DoubleBufferProvider;
using dblbuf::Data;

class ConsumerNode : public rclcpp::Node {
public:
  explicit ConsumerNode(const rclcpp::NodeOptions & options)
  : Node("consumer_node", options)
  {
    buffer_ = DoubleBufferProvider::get();

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "tick", 10,
      std::bind(&ConsumerNode::on_tick, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "ConsumerNode started");
  }

private:
  void on_tick(const std_msgs::msg::String::SharedPtr msg) {
    (void)msg; // 用作触发

    auto front = buffer_->read();
    size_t size = front->bytes.size();

    // 取前 5 个字节的值
    std::ostringstream oss;
    for (size_t i = 0; i < std::min<size_t>(5, size); ++i) {
      oss << static_cast<int>(front->bytes[i]) << " ";
    }

    RCLCPP_INFO(this->get_logger(),
                "Read front size=%zu first5=[%s]",
                size, oss.str().c_str());

    // 在回调的安全点进行交换
    if (buffer_->swap()) {
      RCLCPP_INFO(this->get_logger(), "[DoubleBuffer] front/back 已切换");
    }
  }

  std::shared_ptr<DoubleBuffer> buffer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};