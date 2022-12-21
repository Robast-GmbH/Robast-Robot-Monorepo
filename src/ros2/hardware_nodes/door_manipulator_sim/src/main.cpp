#include <memory>

#include "door_manipulator_sim/door_manipulator_sim.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<door_manipulator_sim::DoorManipulatorSim>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
