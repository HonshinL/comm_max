#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImagePublisher : public rclcpp::Node {
public:
  ImagePublisher()
  : Node("image_publisher"),
    width_(1920), height_(1080), channels_(3), fps_(30) {
    using sensor_msgs::msg::Image;
    rclcpp::QoS qos(10);
    qos.reliable();
    pub_ = this->create_publisher<Image>("big_image", qos);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / fps_),
      [this]() { publish_frame(); });
    buffer_.resize(width_ * height_ * channels_, 123); // 固定内容，便于校验
  }

private:
  void publish_frame() {
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "camera";
    msg.height = height_;
    msg.width = width_;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = width_ * channels_;
    msg.data = buffer_;
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<uint8_t> buffer_;
  int width_, height_, channels_, fps_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}