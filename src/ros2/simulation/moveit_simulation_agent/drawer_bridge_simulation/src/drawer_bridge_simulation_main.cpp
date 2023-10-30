#include "drawer_bridge_simulation/drawer_bridge_simulation.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto drawer_bridge_simulation_node = std::make_shared<drawer_bridge_simulation::DrawerSimulation>();

  rclcpp::spin(drawer_bridge_simulation_node);

  rclcpp::shutdown();

  return 0;
}