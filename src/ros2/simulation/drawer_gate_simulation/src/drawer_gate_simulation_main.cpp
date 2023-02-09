#include "drawer_gate_simulation/drawer_gate_simulation.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto drawer_gate_simulation_node = std::make_shared<drawer_gate_simulation::DrawerSimulation>();

  rclcpp::spin(drawer_gate_simulation_node);

  rclcpp::shutdown();

  return 0;
}