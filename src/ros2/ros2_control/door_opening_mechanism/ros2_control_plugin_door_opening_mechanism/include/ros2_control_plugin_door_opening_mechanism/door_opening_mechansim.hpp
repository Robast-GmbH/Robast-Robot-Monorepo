#ifndef ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_HPP_
#define ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dryve_d1_bridge/d1.hpp"
#include "dryve_d1_bridge/d1_configs.hpp"
#include "dryve_d1_bridge/socket_wrapper.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_plugin_door_opening_mechanism/visibility_control.h"

namespace ros2_control_plugin_door_opening_mechanism
{
  class DoorOpeningMechanismSystemHardware : public hardware_interface::SystemInterface
  {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DoorOpeningMechanismSystemHardware)

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

    ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_PUBLIC hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    std::vector<double> _hw_position_commands;
    std::vector<double> _hw_position_states;

    std::vector<double> _hw_velocity_commands;
    std::vector<double> _hw_velocity_states;

    std::unique_ptr<dryve_d1_bridge::D1> _x_axis;
    std::unique_ptr<dryve_d1_bridge::D1> _y_axis;

    bool _is_start_time_initialized = false;   // TODO@Jacob: Remove this
    rclcpp::Time _start_time;
  };

}   // namespace ros2_control_plugin_door_opening_mechanism

#endif   // ROS2_CONTROL_PLUGIN_DOOR_OPENING_MECHANISM_HPP_
