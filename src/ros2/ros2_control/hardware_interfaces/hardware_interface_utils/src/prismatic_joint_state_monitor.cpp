#include "hardware_interface_utils/prismatic_joint_state_monitor.hpp"

namespace hardware_interface_utils
{
  PrismaticJointStateMonitor::PrismaticJointStateMonitor(const std::string& odom_topic)
      : _odom_topic(odom_topic),
        _trigger_trajectory_execution(false),
        _is_trajectory_execution_in_motion(false),
        Node("prismatic_joint_state_monitor")
  {
    _subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        _odom_topic, 10, std::bind(&PrismaticJointStateMonitor::odom_callback, this, std::placeholders::_1));
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
    if (_trigger_trajectory_execution && !_is_trajectory_execution_in_motion)
    {
      set_initial_position();
      _is_trajectory_execution_in_motion = true;
    }

    if (_is_trajectory_execution_in_motion)
    {
      hw_position_states[0] = compute_prismatic_joint_state();
      hw_velocity_states[0] = get_velocity_state();
    }
  }

  void PrismaticJointStateMonitor::set_trigger_trajectory_execution(bool trigger_trajectory_execution)
  {
    _trigger_trajectory_execution = trigger_trajectory_execution;
  }

  void PrismaticJointStateMonitor::set_is_trajectory_execution_in_motion(bool is_trajectory_execution_in_motion)
  {
    _is_trajectory_execution_in_motion = is_trajectory_execution_in_motion;
  }

}   // namespace hardware_interface_utils
