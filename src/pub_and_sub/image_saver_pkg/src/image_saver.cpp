#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class ImageSaver : public rclcpp::Node {
public:
    ImageSaver() : Node("image_saver") {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ImageSaver::imageCallback, this, std::placeholders::_1));

        last_save_time_ = this->now();
        save_count_ = 0;
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto now = this->now();
        auto elapsed = (now - last_save_time_).seconds();

        if (elapsed >= 10.0) {  // 每10秒保存一次
            try {
                cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
                std::string filename = "saved_image_" + std::to_string(save_count_) + ".jpg";
                cv::imwrite(filename, frame);
                RCLCPP_INFO(this->get_logger(), "保存图片: %s", filename.c_str());

                save_count_++;
                last_save_time_ = now;
            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Time last_save_time_;
    int save_count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaver>());
    rclcpp::shutdown();
    return 0;
}
