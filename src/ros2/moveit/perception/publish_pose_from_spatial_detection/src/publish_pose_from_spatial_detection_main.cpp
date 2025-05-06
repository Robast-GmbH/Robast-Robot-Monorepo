#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "publish_pose_from_spatial_detection/publish_pose_from_spatial_detection.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception::PublishPoseFromSpatialDetection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}