#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher() : Node("camera_publisher") {
        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        // 打开摄像头（默认0号设备）
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开摄像头");
            return;
        }

        // 定时器：每 33ms (~30fps) 采集一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraPublisher::publishFrame, this));
    }

private:
    void publishFrame() {
        cv::Mat frame;
        cap_ >> frame;  // 读取一帧
        if (frame.empty()) return;

        // 转换为 ROS2 Image 消息
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();

        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
