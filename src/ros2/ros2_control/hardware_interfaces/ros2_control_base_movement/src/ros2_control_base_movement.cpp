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

    _is_trajectory_execution_in_motion = false;
    _trigger_trajectory_execution = false;

    _odom_topic = info_.hardware_parameters["odom_topic"];
    if (_odom_topic.empty())
    {
      _odom_topic = "/odom";
    }

    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", "__node:=topic_based_ros2_control_" + info_.name });
    _node = rclcpp::Node::make_shared("_", options);

    return hardware_interface_utils::configure_joints(info.joints, _LOGGER);
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

    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Successfully configured!");

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
    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Activating ...please wait...");

    // command and state should be equal when starting
    for (uint i = 0; i < _hw_position_states.size(); i++)
    {
      _hw_position_commands[i] = _hw_position_states[i];
    }
    for (uint i = 0; i < _hw_velocity_states.size(); i++)
    {
      _hw_velocity_commands[i] = _hw_velocity_states[i];
    }

    _subscriber_odom = _node->create_subscription<nav_msgs::msg::Odometry>(
        _odom_topic, 10, std::bind(&BaseMovementSystemHardware::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Deactivating ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_shutdown(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Shutting down ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Successful shutdown!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_cleanup(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Cleaning up ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseMovementSystemHardware::on_error(
      const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // TODO@Jacob: Check, if this will be triggered some day. Up to the point of working on this, I found no way that
    // TODO@Jacob: on_deactivate, on_cleanup, on_shutdown or on_error are triggered
    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "On Error: Cleaning up ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "On Error: Cleaned up successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BaseMovementSystemHardware::read(const rclcpp::Time& /*time*/,
                                                                   const rclcpp::Duration& /*period*/)
  {
    if (rclcpp::ok())
    {
      rclcpp::spin_some(_node);
    }

    if (_trigger_trajectory_execution && !_is_trajectory_execution_in_motion)
    {
      _initial_position = _latest_odometry_msg.pose.pose.position;
      _is_trajectory_execution_in_motion = true;
    }

    if (_is_trajectory_execution_in_motion)
    {
      _hw_position_states[0] = compute_prismatic_joint_state(_latest_odometry_msg.pose.pose.position);
      _hw_velocity_states[0] = _latest_odometry_msg.twist.twist.linear.x;
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BaseMovementSystemHardware::write(const rclcpp::Time& /*time*/,
                                                                    const rclcpp::Duration& /*period*/)
  {
    if (std::abs(_hw_velocity_commands[0]) > 0.00001)
    {
      // When we receive a velocity command, we know the trajectory execution has started, so the robot is in motion
      // and we want to track the position and velocity of the robot
      RCLCPP_INFO(rclcpp::get_logger(_LOGGER), "Velocity command: %f", _hw_velocity_commands[0]); //TODO: Remove this
      _trigger_trajectory_execution = true;
    }
    else
    {
      _trigger_trajectory_execution = false;
      _is_trajectory_execution_in_motion = false;
    }

    // We don't do anything with the commands here, because we have a chained controller that is responsible for taking
    // the command values of this joint and translating them into the commands for the wheels, so the cmd_vel topic
    return hardware_interface::return_type::OK;
  }

  void BaseMovementSystemHardware::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // TODO: Do we need some kind of real time buffer here?
    _latest_odometry_msg = *msg;
  }

  float BaseMovementSystemHardware::compute_prismatic_joint_state(const geometry_msgs::msg::Point current_odom_positon)
  {
    float delta_x = current_odom_positon.x - _initial_position.x;
    float delta_y = current_odom_positon.y - _initial_position.y;

    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
  }

}   // namespace ros2_control_base_movement

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_base_movement::BaseMovementSystemHardware, hardware_interface::SystemInterface)
