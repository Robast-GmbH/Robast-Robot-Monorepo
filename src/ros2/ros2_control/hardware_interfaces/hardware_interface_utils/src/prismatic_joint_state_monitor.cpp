#include "hardware_interface_utils/prismatic_joint_state_monitor.hpp"

namespace hardware_interface_utils
{
  PrismaticJointStateMonitor::PrismaticJointStateMonitor(const std::string& odom_topic)
      : _odom_topic(odom_topic),
        _initial_position_initialized(false),
        _is_trajectory_execution_in_motion(false),
        _reset_initial_position(false),
        Node("prismatic_joint_state_monitor")
  {
    _odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        _odom_topic, 10, std::bind(&PrismaticJointStateMonitor::odom_callback, this, std::placeholders::_1));

    _reset_initial_position_subscriber = this->create_subscription<std_msgs::msg::Empty>(
        "reset_initial_position_for_mobile_base",
        10,
        std::bind(&PrismaticJointStateMonitor::reset_initial_position, this, std::placeholders::_1));
  }

  void PrismaticJointStateMonitor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // TODO: Do we need some kind of real time buffer here?
    _latest_odom_msg = *msg;
  }

  float PrismaticJointStateMonitor::compute_prismatic_joint_state()
  {
    float delta_x = _latest_odom_msg.pose.pose.position.x - _initial_position.x;
    float delta_y = _latest_odom_msg.pose.pose.position.y - _initial_position.y;

    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
  }

  double PrismaticJointStateMonitor::get_velocity_state() const
  {
    return _latest_odom_msg.twist.twist.linear.x;
  }

  void PrismaticJointStateMonitor::set_initial_position()
  {
    _initial_position = _latest_odom_msg.pose.pose.position;
  }

  void PrismaticJointStateMonitor::update_state(std::vector<double>& hw_position_states,
                                                std::vector<double>& hw_velocity_states)
  {
    if (_is_trajectory_execution_in_motion)
    {
      if (!_initial_position_initialized)
      {
        set_initial_position();
        _initial_position_initialized = true;
      }

      hw_position_states[0] = compute_prismatic_joint_state();
      hw_velocity_states[0] = get_velocity_state();
    }
    else
    {
      if (_reset_initial_position)
      {
        _reset_initial_position = false;
        hw_position_states[0] = 0.0;
        hw_velocity_states[0] = 0.0;
      }
    }
  }

  void PrismaticJointStateMonitor::set_initial_position_initialized(bool initial_position_initialized)
  {
    _initial_position_initialized = initial_position_initialized;
  }

  void PrismaticJointStateMonitor::set_is_trajectory_execution_in_motion(bool is_trajectory_execution_in_motion)
  {
    _is_trajectory_execution_in_motion = is_trajectory_execution_in_motion;
  }

  void PrismaticJointStateMonitor::reset_initial_position(const std_msgs::msg::Empty::SharedPtr msg)
  {
    _initial_position_initialized = false;
    _reset_initial_position = true;
  }

}   // namespace hardware_interface_utils
