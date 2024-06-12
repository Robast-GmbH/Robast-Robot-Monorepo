#ifndef HARDWARE_INTERFACE_UTILS__PRISMATIC_JOINT_STATE_MONITOR_HPP_
#define HARDWARE_INTERFACE_UTILS__PRISMATIC_JOINT_STATE_MONITOR_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware_interface_utils
{
  class PrismaticJointStateMonitor : public rclcpp::Node
  {
   public:
    PrismaticJointStateMonitor(const std::string& odom_topic);

    void update_state(std::vector<double>& hw_position_states, std::vector<double>& hw_velocity_states);

    void set_trigger_trajectory_execution(bool trigger_trajectory_execution);

    void set_is_trajectory_execution_in_motion(bool is_trajectory_execution_in_motion);

   private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber_odom;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::string _odom_topic;

    nav_msgs::msg::Odometry _latest_odom_msg;

    geometry_msgs::msg::Point _initial_position;

    bool _trigger_trajectory_execution;
    bool _is_trajectory_execution_in_motion;

    double get_velocity_state() const;

    float compute_prismatic_joint_state();

    void set_initial_position();
  };
}   // namespace hardware_interface_utils

#endif   // HARDWARE_INTERFACE_UTILS__PRISMATIC_JOINT_STATE_MONITOR_HPP_