#ifndef GZ_ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_
#define GZ_ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_

#include "gz_ros2_control/gz_system_interface.hpp"
#include "gz_ros2_control_base_movement/sim_ros2_control_base_movement.hpp"

namespace gz_ros2_control_base_movement
{
  // These class must inherit `gz_ros2_control::GazeboSimSystemInterface` which implements a
  // simulated `ros2_control` `hardware_interface::SystemInterface`.

  class GzBaseMovementSystemHardware : public SimBaseMovement<gz_ros2_control::GazeboSimSystemInterface>
  {
    // Documentation Inherited
    bool initSim(rclcpp::Node::SharedPtr& model_nh,
                 std::map<std::string, sim::Entity>& joints,
                 const hardware_interface::HardwareInfo& hardware_info,
                 sim::EntityComponentManager& _ecm,
                 unsigned int update_rate) override;
  };

}   // namespace gz_ros2_control_base_movement

#endif   // GZ_ROS2_CONTROL_BASE_MOVEMENT__GZ_SYSTEM_HPP_
