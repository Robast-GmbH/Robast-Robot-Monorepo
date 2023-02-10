#include "door_opening_mechanism_simulation/door_opening_mechanism_simulation.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto door_opening_mechanism_simulation_node =
      std::make_shared<door_opening_mechanism_simulation::DoorMechanismSimulation>();

  rclcpp::spin(door_opening_mechanism_simulation_node);

  rclcpp::shutdown();

  return 0;
}