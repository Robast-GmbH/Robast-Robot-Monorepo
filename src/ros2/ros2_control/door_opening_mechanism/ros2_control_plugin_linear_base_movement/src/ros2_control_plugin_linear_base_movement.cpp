#include "ros2_control_plugin_linear_base_movement/ros2_control_plugin_linear_base_movement.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_plugin_linear_base_movement
{
  hardware_interface::CallbackReturn LinearBaseMovementSystemHardware::on_init(
      const hardware_interface::HardwareInfo& info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    _hw_position_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {
      RCLCPP_INFO(
          rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Configuring joint '%s'.", joint.name.c_str());

      // LinearBaseMovementSystemHardware has position and velocity state and command interface on each joint
      if (joint.command_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("LinearBaseMovementSystemHardware"),
                     "Joint '%s' has %zu command interfaces found. 2 or less expected.",
                     joint.name.c_str(),
                     joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("LinearBaseMovementSystemHardware"),
                     "Joint '%s' have %s command interfaces found. '%s' or '%s' expected.",
                     joint.name.c_str(),
                     joint.command_interfaces[0].name.c_str(),
                     hardware_interface::HW_IF_POSITION,
                     hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("LinearBaseMovementSystemHardware"),
                     "Joint '%s' has %zu state interface. 2 or less expected.",
                     joint.name.c_str(),
                     joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("LinearBaseMovementSystemHardware"),
                     "Joint '%s' have %s state interface. '%s' or '%s' expected.",
                     joint.name.c_str(),
                     joint.state_interfaces[0].name.c_str(),
                     hardware_interface::HW_IF_POSITION,
                     hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LinearBaseMovementSystemHardware::on_configure(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // reset values always when configuring hardware
    for (uint i = 0; i < _hw_position_states.size(); i++)
    {
      _hw_position_states[i] = 0;
      _hw_position_commands[i] = 0;
    }
    for (uint i = 0; i < _hw_velocity_states.size(); i++)
    {
      _hw_velocity_states[i] = 0;
      _hw_velocity_commands[i] = 0;
    }

    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> LinearBaseMovementSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_hw_position_states[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_hw_velocity_states[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> LinearBaseMovementSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_hw_position_commands[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_hw_velocity_commands[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn LinearBaseMovementSystemHardware::on_activate(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Activating ...please wait...");

    // command and state should be equal when starting
    for (uint i = 0; i < _hw_position_states.size(); i++)
    {
      _hw_position_commands[i] = _hw_position_states[i];
    }
    for (uint i = 0; i < _hw_velocity_states.size(); i++)
    {
      _hw_velocity_commands[i] = _hw_velocity_states[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LinearBaseMovementSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Deactivating ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LinearBaseMovementSystemHardware::on_shutdown(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Shutting down ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Successful shutdown!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LinearBaseMovementSystemHardware::on_cleanup(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Cleaning up ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn LinearBaseMovementSystemHardware::on_error(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "On Error: Cleaning up ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"), "On Error: Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type LinearBaseMovementSystemHardware::read(const rclcpp::Time& /*time*/,
                                                                         const rclcpp::Duration& /*period*/)
  {
    // TODO@Jacob: Implement read from hardware
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type LinearBaseMovementSystemHardware::write(const rclcpp::Time& /*time*/,
                                                                          const rclcpp::Duration& /*period*/)
  {
    if (_hw_velocity_commands[0] != 0.0)
    {
      RCLCPP_INFO(rclcpp::get_logger("LinearBaseMovementSystemHardware"),
                  "Received velocity command %f for linear base movement.",
                  _hw_velocity_commands[0]);
    }
    // TODO@Jacob: Implement write to hardware
    return hardware_interface::return_type::OK;
  }

}   // namespace ros2_control_plugin_linear_base_movement

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_plugin_linear_base_movement::LinearBaseMovementSystemHardware,
                       hardware_interface::SystemInterface)
