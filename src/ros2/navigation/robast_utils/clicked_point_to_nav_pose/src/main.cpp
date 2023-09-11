#include <memory>

#include "clicked_point_to_nav_pose/clicked_point_to_nav_pose.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robast_utils::ClickedPointToNavPose>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}