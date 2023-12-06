#ifndef ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_HPP_
#define ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_HPP_

#include <memory>
#include <string>
#include <vector>

// #include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "include/gz_ros2_control/gz_system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_plugin_linear_base_movement_gazebo/visibility_control.h"

namespace ros2_control_plugin_linear_base_movement_gazebo
{
  class LinearBaseMovementSystemHardware : public gz_ros2_control::GazeboSimSystemInterface
  {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(LinearBaseMovementSystemHardware)

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_PUBLIC hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    std::vector<double> _hw_position_commands;
    std::vector<double> _hw_position_states;

    std::vector<double> _hw_velocity_commands;
    std::vector<double> _hw_velocity_states;
  };

}   // namespace ros2_control_plugin_linear_base_movement_gazebo

#endif   // ROS2_CONTROL_PLUGIN_LINEAR_BASE_MOVEMENT_GAZEBO_HPP_
