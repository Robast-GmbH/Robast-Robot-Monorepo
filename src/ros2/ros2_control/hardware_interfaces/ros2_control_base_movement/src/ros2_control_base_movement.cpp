#include "ros2_control_base_movement/ros2_control_base_movement.hpp"

namespace ros2_control_base_movement
{
  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
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
      RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Configuring joint '%s'.", joint.name.c_str());

      // BaseMovementSystemHardware has position and velocity state and command interface on each joint
      if (joint.command_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("BaseMovementSystemHardware"),
                     "Joint '%s' has %zu command interfaces found. 2 or less expected.",
                     joint.name.c_str(),
                     joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("BaseMovementSystemHardware"),
                     "Joint '%s' have %s command interfaces found. '%s' or '%s' expected.",
                     joint.name.c_str(),
                     joint.command_interfaces[0].name.c_str(),
                     hardware_interface::HW_IF_POSITION,
                     hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() > 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger("BaseMovementSystemHardware"),
                     "Joint '%s' has %zu state interface. 2 or less expected.",
                     joint.name.c_str(),
                     joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if ((joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) &&
          (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger("BaseMovementSystemHardware"),
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

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_configure(
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

    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> BaseMovementSystemHardware::export_state_interfaces()
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

  std::vector<hardware_interface::CommandInterface> BaseMovementSystemHardware::export_command_interfaces()
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

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_activate(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Activating ...please wait...");

    // command and state should be equal when starting
    for (uint i = 0; i < _hw_position_states.size(); i++)
    {
      _hw_position_commands[i] = _hw_position_states[i];
    }
    for (uint i = 0; i < _hw_velocity_states.size(); i++)
    {
      _hw_velocity_commands[i] = _hw_velocity_states[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Deactivating ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_shutdown(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Shutting down ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Successful shutdown!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_cleanup(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Cleaning up ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_error(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "On Error: Cleaning up ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("BaseMovementSystemHardware"), "On Error: Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BaseMovementSystemHardware::read(const rclcpp::Time& /*time*/,
                                                                   const rclcpp::Duration& /*period*/)
  {
    // Right now, this is kind of a mocked implementation. Right now, I am not sure if we need the feedback for the
    // mobile_base joint. It might be enough that we have this joint for planning and executing the base movement open
    // loop. If we need the feedback, we need to implement this here.
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BaseMovementSystemHardware::write(const rclcpp::Time& /*time*/,
                                                                    const rclcpp::Duration& /*period*/)
  {
    // We don't do anything with the commands here, because we have a chained controller that is responsible for taking
    // the command values of this joint and translating them into the commands for the wheels, so the cmd_vel topic
    return hardware_interface::return_type::OK;
  }

}   // namespace ros2_control_base_movement

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_base_movement::BaseMovementSystemHardware, hardware_interface::SystemInterface)
