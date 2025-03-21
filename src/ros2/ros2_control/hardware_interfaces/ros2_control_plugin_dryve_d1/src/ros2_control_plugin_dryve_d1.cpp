#include "ros2_control_plugin_dryve_d1/ros2_control_plugin_dryve_d1.hpp"

namespace ros2_control_plugin_dryve_d1
{
  hardware_interface::CallbackReturn DryveD1SystemHardware::on_init(const hardware_interface::HardwareInfo& info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    _ip_address = info_.hardware_parameters["ip_address"];
    _port = stod(info_.hardware_parameters["port"]);
    _direction = stod(info_.hardware_parameters["direction"]);
    _si_unit_factor = stod(info_.hardware_parameters["si_unit_factor"]);
    _is_prismatic_joint = info_.hardware_parameters["joint_type"] == "prismatic";
    _zero_velocity_published_last = false;

    _hw_position_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_velocity_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    return hardware_interface_utils::configure_joints(info.joints, _logger);
  }

  hardware_interface::CallbackReturn DryveD1SystemHardware::on_configure(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // Please mind! In order to get a connection to the dryve d1, there need to be a port forward on the router from the
    // port configured in DRYVE_D1_IP_ADDRESS_X_AXIS to the Port configured for Modbus TCP in the dryve d1 (usually 502)
    RCLCPP_INFO(
        rclcpp::get_logger(_logger), "Configuring dryve d1 for ip address %s and port %d", _ip_address.c_str(), _port);
    _dryve_d1 =
        std::make_unique<dryve_d1_bridge::D1>(_ip_address, _port, std::make_unique<dryve_d1_bridge::SocketWrapper>());

    // Please mind! The state_machine only runs through if you made the following settings on the dryve d1:
    // (1) Drive Profile -> Modbus TCP Gateway
    // (2) Inputs/Outputs -> Enable
    // (3) Communication -> Port = 502
    // (4) Communication -> Modbus TCP Gateway ON
    // If these settings have already been made and once power was switched on and off you have to disable and
    // enable the Modbus TCP Gateway once in order to be able to connect to the dryve d1 again
    // TODO@Jacob: Find a possibility that Modbus TCP Gateway does not need to be switched on and off after power up
    _dryve_d1->run_dryve_state_machine();

    _dryve_d1->set_si_unit_factor(
        dryve_d1_bridge::map_si_unit_factor_double_to_char(_si_unit_factor, _is_prismatic_joint), _is_prismatic_joint);

    _dryve_d1->set_debug_mode_off();

    RCLCPP_INFO(rclcpp::get_logger(_logger), "Start homing now ...");
    _dryve_d1->start_dryve_homing(
        dryve_d1_bridge::HOMING_VELOCITY, dryve_d1_bridge::HOMING_VELOCITY, dryve_d1_bridge::HOMING_ACCELERATION);

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

    RCLCPP_INFO(rclcpp::get_logger(_logger), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DryveD1SystemHardware::export_state_interfaces()
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

  std::vector<hardware_interface::CommandInterface> DryveD1SystemHardware::export_command_interfaces()
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

  hardware_interface::CallbackReturn DryveD1SystemHardware::on_activate(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger(_logger), "Activating ...please wait...");

    // command and state should be equal when starting
    for (uint i = 0; i < _hw_position_states.size(); i++)
    {
      _hw_position_commands[i] = _hw_position_states[i];
    }
    for (uint i = 0; i < _hw_velocity_states.size(); i++)
    {
      _hw_velocity_commands[i] = _hw_velocity_states[i];
    }

    RCLCPP_INFO(rclcpp::get_logger(_logger), "Successfully activated joint '%s'.", info_.joints[0].name.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DryveD1SystemHardware::on_deactivate(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger(_logger), "Deactivating ...please wait...");

    _dryve_d1->close_connection();

    RCLCPP_INFO(rclcpp::get_logger(_logger), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DryveD1SystemHardware::on_shutdown(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger(_logger), "Shutting down ...please wait...");

    _dryve_d1->close_connection();

    RCLCPP_INFO(rclcpp::get_logger(_logger), "Successful shutdown!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DryveD1SystemHardware::on_cleanup(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger(_logger), "Cleaning up ...please wait...");

    _dryve_d1->close_connection();

    RCLCPP_INFO(rclcpp::get_logger(_logger), "Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DryveD1SystemHardware::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger(_logger), "On Error: Cleaning up ...please wait...");

    _dryve_d1->close_connection();

    RCLCPP_INFO(rclcpp::get_logger(_logger), "On Error: Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  void DryveD1SystemHardware::run_dryve_state_machine_after_error(std::string_view dryve_error)
  {
    RCLCPP_WARN(rclcpp::get_logger(_logger),
                "Dryve Error '%s' for joint %s. Running through dryve d1 state machine setup now.",
                dryve_error.data(),
                info_.joints[0].name.c_str());
    _dryve_d1->run_dryve_state_machine();
  }

  bool DryveD1SystemHardware::is_dryve_error_present()
  {
    std::string_view dryve_error = _dryve_d1->check_for_dryve_error();

    if (dryve_error.empty())
    {
      return false;
    }

    if (dryve_error == _dryve_d1->ERROR_MESSAGE_E12_LIMIT_SWITCH)
    {
      run_dryve_state_machine_after_error(dryve_error);
      return false;
    }
    else if (dryve_error == _dryve_d1->ERROR_MESSAGE_E08_LOAD_SUPPLY_LOW)
    {
      // TODO@Jacob: At some point we might need to find a better way how to handle this error
      return true;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger(_logger),
                   "Dryve Error '%s' for joint %s",
                   dryve_error.data(),
                   info_.joints[0].name.c_str());
      return true;
    }
  }

  hardware_interface::return_type DryveD1SystemHardware::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
  {
    // Please mind:
    // Setting the _si_unit_factor to the dryve d1 works in principle. If you check the displayed position values in the
    // web interface, the values change when you change the si_unit_factor.
    // BUT: No matter what si_unit_factor you set for the dryve d1, we get the same values from the tcp connection!
    // Therefore we have to divide and multiply the si_unit_factor by hand in order to get the correct values
    double hw_position_state =
        static_cast<double>(_dryve_d1->read_object_value(_dryve_d1->OBJECT_INDEX_1_READ_POSITION_ACTUAL_VALUE,
                                                         _dryve_d1->OBJECT_INDEX_2_READ_POSITION_ACTUAL_VALUE)) /
        _si_unit_factor * _direction;

    double hw_velocity_states =
        static_cast<double>(_dryve_d1->read_object_value(_dryve_d1->OBJECT_INDEX_1_READ_VELOCITY_ACTUAL_VALUE,
                                                         _dryve_d1->OBJECT_INDEX_2_READ_VELOCITY_ACTUAL_VALUE)) /
        _si_unit_factor * _direction;

    if (_is_prismatic_joint)
    {
      _hw_position_states[0] = dryve_d1_bridge::mm_to_m(hw_position_state);
      _hw_velocity_states[0] = dryve_d1_bridge::mm_to_m(hw_velocity_states);
    }
    else
    {
      _hw_position_states[0] = dryve_d1_bridge::degree_to_rad(hw_position_state);
      _hw_velocity_states[0] = dryve_d1_bridge::degree_to_rad(hw_velocity_states);
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type DryveD1SystemHardware::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/)
  {
    if (std::abs(_hw_velocity_commands[0]) > dryve_d1_bridge::MIN_VELOCITY)
    {
      if (is_dryve_error_present())
      {
        return hardware_interface::return_type::ERROR;
      }

      _dryve_d1->set_profile_velocity(_hw_velocity_commands[0] * _si_unit_factor * _direction,
                                      dryve_d1_bridge::ACCELERATION,
                                      dryve_d1_bridge::DECELERATION);
      _zero_velocity_published_last = false;
    }
    else
    {
      if (!_zero_velocity_published_last)
      {
        _dryve_d1->set_profile_velocity(0, dryve_d1_bridge::ACCELERATION, dryve_d1_bridge::DECELERATION);
        _zero_velocity_published_last = true;
      }
    }

    return hardware_interface::return_type::OK;
  }

}   // namespace ros2_control_plugin_dryve_d1

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_plugin_dryve_d1::DryveD1SystemHardware, hardware_interface::SystemInterface)
