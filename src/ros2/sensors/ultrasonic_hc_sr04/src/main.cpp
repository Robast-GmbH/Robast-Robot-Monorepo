#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ultrasonic_hc_sr04/ultrasonic_hc_sr04.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ultrasonic_sensor::UltrasonicHCSR04>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
