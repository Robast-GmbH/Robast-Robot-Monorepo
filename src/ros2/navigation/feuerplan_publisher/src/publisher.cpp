#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class FeuerplanPublisher : public rclcpp::Node
{
  public:
  FeuerplanPublisher() : Node("feuerplan_publisher"), publisher_(nullptr)
    {
      this->declare_parameter("feuerplan_path","");

      std::string feuerplan_path = this->get_parameter("feuerplan_path").as_string();

      rclcpp::QoS map_qos(10); 
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1);

      publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("global_costmap/feuerplan_image_topic",map_qos);

      cv::Mat feuerplan_image = cv::imread(feuerplan_path,cv::IMREAD_GRAYSCALE);
      preprocessImage(feuerplan_image);
      publishOccupancyGrid(feuerplan_image);
    }
      

     void preprocessImage(cv::Mat& image)
    {
        cv::Mat processed_image;
        cv::GaussianBlur(image, processed_image, cv::Size(3, 3), 3, 3);
        cv::Canny(processed_image, processed_image, 50, 150, 3);

        int dilation_size = 1;
        cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));
        cv::dilate(processed_image, processed_image, dilation_element);

        image = image & processed_image;
        cv::threshold(image, image, 150, 127, cv::THRESH_BINARY);
    }

    void publishOccupancyGrid(const cv::Mat& feuerplan_image)
    {
        std::vector<int8_t> data;
        data.reserve(feuerplan_image.rows * feuerplan_image.cols);

        for (int i = 0; i < feuerplan_image.rows; ++i) {
            for (int j = 0; j < feuerplan_image.cols; ++j) {
                data.emplace_back(feuerplan_image.at<uchar>(i, j));
            }
        }

        // This is a temporary solution as the origin's orientation and positon should not be hardcoded.
        // TODO the origin's orientation and positon should be obtained from the transformation between feuerplan and map.
        auto ros_image_msg = nav_msgs::msg::OccupancyGrid();
        ros_image_msg.header.stamp = this->now();
        ros_image_msg.header.frame_id = "feuerplan_map";
        ros_image_msg.info.map_load_time.sec = 0;
        ros_image_msg.info.map_load_time.nanosec = 0;
        ros_image_msg.info.resolution = 0.05;
        ros_image_msg.info.width = feuerplan_image.cols;
        ros_image_msg.info.height = feuerplan_image.rows;
        ros_image_msg.info.origin.position.x = -20.2;
        ros_image_msg.info.origin.position.y = -3.3;
        ros_image_msg.info.origin.position.z = 0.0;
        ros_image_msg.info.origin.orientation.x = 1.0;
        ros_image_msg.info.origin.orientation.y = 0.0;
        ros_image_msg.info.origin.orientation.z = 0.0;
        ros_image_msg.info.origin.orientation.w = 0.0;
        ros_image_msg.data = data;

        RCLCPP_INFO(this->get_logger(), "Feuerplan published");
        publisher_->publish(ros_image_msg);
    }  
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