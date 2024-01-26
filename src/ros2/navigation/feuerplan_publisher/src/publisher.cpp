#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class FeuerplanPublisher : public rclcpp::Node
{
  public:
    FeuerplanPublisher()
    : Node("feuerplan_publisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("feuerplan_image_topic", 10);
      timer_ = this->create_wall_timer(10ms, std::bind(&FeuerplanPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      cv::Mat feuerplan_image = cv::imread("src/navigation/feuerplan_publisher/resources/match_2.jpg", cv::IMREAD_GRAYSCALE);

      std::vector<int8_t> data;
        for (int i = 0; i < feuerplan_image.rows; ++i) {
            for (int j = 0; j < feuerplan_image.cols; ++j) {
                data.push_back(feuerplan_image.at<uchar>(i, j));
            }
        }

    //  RCLCPP_INFO(this->get_logger(), "%d", data)
      auto ros_image_msg = nav_msgs::msg::OccupancyGrid();
      ros_image_msg.header.stamp = this->now();
      ros_image_msg.header.frame_id = "map";
      ros_image_msg.info.map_load_time.sec = 0;
      ros_image_msg.info.map_load_time.nanosec = 0;
      ros_image_msg.info.resolution = 0.05;
      ros_image_msg.info.width = feuerplan_image.cols;
      ros_image_msg.info.height = feuerplan_image.rows;
      ros_image_msg.info.origin.position.x = -17.02;
      ros_image_msg.info.origin.position.y = -8.72;
      ros_image_msg.info.origin.position.z = 0.0;
      ros_image_msg.info.origin.orientation.x = 1.0;
      ros_image_msg.info.origin.orientation.y = 1.0;
      ros_image_msg.info.origin.orientation.z = 1.0;
      ros_image_msg.info.origin.orientation.w = 0.0;
      ros_image_msg.data = data;
      publisher_->publish(ros_image_msg);
      RCLCPP_INFO(this->get_logger(), "Grayscale Image published");

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeuerplanPublisher>());
  rclcpp::shutdown();
  return 0;
}