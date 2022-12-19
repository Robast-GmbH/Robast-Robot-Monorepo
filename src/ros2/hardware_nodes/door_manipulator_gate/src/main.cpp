#include <memory>

#include "door_manipulator_gate/door_manipulator_gate.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<door_manipulator_gate::DoorManipulatorGate>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
