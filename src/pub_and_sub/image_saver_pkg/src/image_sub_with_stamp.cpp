#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "image_saver_pkg/perf_monitor.hpp"

class ImageSubscriber : public rclcpp::Node {
public:
  ImageSubscriber() : Node("image_subscriber") {
    using sensor_msgs::msg::Image;
    rclcpp::QoS qos(10);
    qos.reliable();
    sub_ = this->create_subscription<Image>(
      "big_image", qos,
      [this](const Image::SharedPtr msg) { on_frame(msg); });
    start_time_ = this->get_clock()->now();
  }

private:
  void on_frame(const sensor_msgs::msg::Image::SharedPtr & msg) {
    auto now = this->get_clock()->now();
    double latency_ms = (now - msg->header.stamp).seconds() * 1000.0;
    frames_++;
    bytes_total_ += msg->data.size();

    auto elapsed = (now - start_time_).seconds();
    if (elapsed >= 1.0) {
      double fps = frames_ / elapsed;
      double mbps = (bytes_total_ / (1024.0 * 1024.0)) / elapsed;

      RCLCPP_INFO(this->get_logger(),
        "Latency avg: %.2f ms | FPS: %.1f | Throughput: %.2f MB/s | Size: %.2f MB | %s",
        latency_sum_ / frames_, fps, mbps,
        msg->data.size() / (1024.0 * 1024.0),
        perf_.sample().c_str());

      start_time_ = now;
      frames_ = 0;
      bytes_total_ = 0;
      latency_sum_ = 0.0;
    } else {
      latency_sum_ += latency_ms;
    }
  }

  PerfMonitor perf_;   // 性能监控对象
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Time start_time_;
  size_t frames_{0};
  double latency_sum_{0.0};
  size_t bytes_total_{0};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}