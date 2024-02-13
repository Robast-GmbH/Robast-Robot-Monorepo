#ifndef IGN_ROS2_CONTROL_BASE_MOVEMENT__IGN_SYSTEM_HPP_
#define IGN_ROS2_CONTROL_BASE_MOVEMENT__IGN_SYSTEM_HPP_

#include "gz_ros2_control_base_movement/sim_ros2_control_base_movement.hpp"
#include "ign_ros2_control/ign_system_interface.hpp"

namespace ign_ros2_control_base_movement
{
  // These class must inherit `ign_ros2_control::GazeboSimSystemInterface` which implements a
  // simulated `ros2_control` `hardware_interface::SystemInterface`.

  class IgnBaseMovementSystemHardware : public SimBaseMovement<ign_ros2_control::IgnitionSystemInterface>
  {
    // Documentation Inherited
    bool initSim(rclcpp::Node::SharedPtr& model_nh,
                 std::map<std::string, ignition::gazebo::Entity>& joints,
                 const hardware_interface::HardwareInfo& hardware_info,
                 ignition::gazebo::EntityComponentManager& _ecm,
                 int& update_rate) override;
  };

}   // namespace ign_ros2_control_base_movement

#endif   // IGN_ROS2_CONTROL_BASE_MOVEMENT__IGN_SYSTEM_HPP_
