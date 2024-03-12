#include <cv_bridge/cv_bridge.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <functional>

class camera_node : public rclcpp::Node {
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      image_capture_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr edge_publisher_;

  void
  capture_callback(const sensor_msgs::msg::Image::SharedPtr ros_img) const {
    auto cv_img =
        cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(cv_img->image, cv_img->image, cv::Size{3, 3}, 0.5);
    cv::Canny(cv_img->image, cv_img->image, 0, 300);
    cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_GRAY2BGR);
    RCLCPP_INFO(this->get_logger(), "publishing to edges_node");
    edge_publisher_->publish(*cv_img->toImageMsg());
  }

public:
  camera_node()
      : Node{"camera"},
        image_capture_subscriber_{
            this->create_subscription<sensor_msgs::msg::Image>(
                "/camera_sensor/image_raw", 10,
                std::bind(&camera_node::capture_callback, this,
                          std::placeholders::_1))},
        edge_publisher_{
            this->create_publisher<sensor_msgs::msg::Image>("edges_node", 10)} {
    RCLCPP_INFO(this->get_logger(), "edge detector is up");
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camera_node>());
  rclcpp::shutdown();
}
