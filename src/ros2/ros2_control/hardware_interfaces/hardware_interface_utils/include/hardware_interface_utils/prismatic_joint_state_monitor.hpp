#ifndef HARDWARE_INTERFACE_UTILS__PRISMATIC_JOINT_STATE_MONITOR_HPP_
#define HARDWARE_INTERFACE_UTILS__PRISMATIC_JOINT_STATE_MONITOR_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

namespace hardware_interface_utils
{
  class PrismaticJointStateMonitor : public rclcpp::Node
  {
  public:
    PrismaticJointStateMonitor(const std::string &odom_topic, const bool reset_position_state_after_each_trajectory);

    void update_state(std::vector<double> &hw_position_states, std::vector<double> &hw_velocity_states);

    void set_initial_position_initialized(bool trigger_trajectory_execution);

    void set_is_trajectory_execution_in_motion(bool is_trajectory_execution_in_motion);

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _reset_initial_position_subscriber;

    std::string _odom_topic;
    bool _reset_position_state_after_each_trajectory;

    nav_msgs::msg::Odometry _latest_odom_msg;
    double _summed_up_vx;

    geometry_msgs::msg::Point _initial_position;

    bool _initial_position_initialized;
    bool _is_trajectory_execution_in_motion;
    bool _reset_initial_position;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    double get_velocity_state() const;

    double compute_prismatic_joint_state();

    void set_initial_position();

    void reset_initial_position(const std_msgs::msg::Empty::SharedPtr msg);

    bool is_moving_forward();
  };
} // namespace hardware_interface_utils

#endif // HARDWARE_INTERFACE_UTILS__PRISMATIC_JOINT_STATE_MONITOR_HPP_