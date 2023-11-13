#include <memory>

#include "dryve_d1_bridge/dryve_d1_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dryve_d1_bridge::DryveD1Gate>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
