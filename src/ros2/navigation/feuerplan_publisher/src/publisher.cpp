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
      rclcpp::QoS map_qos(10); 
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1);

      publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("global_costmap/feuerplan_image_topic",map_qos);

      cv::Mat feuerplan_image = cv::imread("src/navigation/feuerplan_publisher/resources/matched.jpg",cv::IMREAD_GRAYSCALE);
      cv::Mat feuerplan_image_smooth;
      cv::Mat feuerplan_image_thresh;

      //cv::threshold(feuerplan_image,feuerplan_image_thresh, 128, 1, cv::THRESH_BINARY | cv::THRESH_OTSU);
      cv::GaussianBlur(feuerplan_image, feuerplan_image_thresh, cv::Size(0, 0), 3);
      cv::addWeighted(feuerplan_image, 1.5, feuerplan_image_thresh, -0.5, 0, feuerplan_image_thresh);
      std::vector<int8_t> data;
      for (int i = 0; i < feuerplan_image_thresh.rows; ++i) {
        for (int j = 0; j < feuerplan_image_thresh.cols; ++j) {
          data.push_back(feuerplan_image_thresh.at<uchar>(i, j));
        }
      }


    //  RCLCPP_INFO(this->get_logger(), "%d", data)
      auto ros_image_msg = nav_msgs::msg::OccupancyGrid();
      ros_image_msg.header.stamp = this->now();
      ros_image_msg.header.frame_id = "map";
      ros_image_msg.info.map_load_time.sec = 0;
      ros_image_msg.info.map_load_time.nanosec = 0;
      ros_image_msg.info.resolution = 0.05000000074505806;
      ros_image_msg.info.width = feuerplan_image.cols;
      ros_image_msg.info.height = feuerplan_image.rows;
      ros_image_msg.info.origin.position.x = -20;
      ros_image_msg.info.origin.position.y = -3.3;
      ros_image_msg.info.origin.position.z = 0.0;
      ros_image_msg.info.origin.orientation.x = 0.0;
      ros_image_msg.info.origin.orientation.y = 0.0;
      ros_image_msg.info.origin.orientation.z = 0.0;
      ros_image_msg.info.origin.orientation.w = 1.0;
      ros_image_msg.data = data;
      RCLCPP_INFO(this->get_logger(), "Feuerplan published");
      publisher_->publish(ros_image_msg);

       //publishLoop();
    }
    //rclcpp::TimerBase::SharedPtr timer_;
    private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeuerplanPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}