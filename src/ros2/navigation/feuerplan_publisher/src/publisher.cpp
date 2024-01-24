#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class FeuerplanPublisher : public rclcpp::Node
{
  public:
    FeuerplanPublisher()
    : Node("feuerplan_publisher")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&FeuerplanPublisher::timer_callback, this));
    }

private:
    void timer_callback() {

        // Create a sample image (replace with your own image loading logic)
        cv::Mat image = cv::imread("src/navigation/feuerplan_publisher/resources/matched.jpg", cv::IMREAD_GRAYSCALE);
        // Convert OpenCV image to ROS sensor_msgs::Image
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();

        for(int i = 0; i<msg.data.size; i++){
          RCLCPP_INFO(node->get_logger(), " [%d]", msg.data[i].c_str());
        }

        // Publish the message
        publisher_->publish(*msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeuerplanPublisher>());
    rclcpp::shutdown();
    return 0;
}