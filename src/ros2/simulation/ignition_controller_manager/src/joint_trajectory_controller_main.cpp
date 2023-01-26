#include <rclcpp/rclcpp.hpp>
#include "ignition_controller_manager/joint_trajectory_controller.hpp"

int main(int argc, char* argv[])
{
    //creat ros2 node
    rclcpp::init(argc, argv);
    
    // create controller
    auto joint_trajectory_controller = std::make_shared<ignition_controller_manager::JointTrajectoryController>();
    // run node until it's exited
    rclcpp::spin(joint_trajectory_controller);
    //clean up
    rclcpp::shutdown();
    return 0;
}