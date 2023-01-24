#include "drawer_simulation/drawer_simulation.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto drawer_simulation_node = std::make_shared<drawer_simulation::DrawerSimulation>();

    // run node until it's exited
    rclcpp::spin(drawer_simulation_node);
    //clean up
    rclcpp::shutdown();
    
    return 0;
}