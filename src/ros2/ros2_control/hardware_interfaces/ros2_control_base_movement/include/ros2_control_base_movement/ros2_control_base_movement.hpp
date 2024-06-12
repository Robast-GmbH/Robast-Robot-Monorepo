#ifndef ROS2_CONTROL_BASE_MOVEMENT_HPP_
#define ROS2_CONTROL_BASE_MOVEMENT_HPP_

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface_utils/hardware_interface_utils.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_base_movement/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace ros2_control_base_movement
{
  class BaseMovementSystemHardware : public hardware_interface::SystemInterface
  {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(BaseMovementSystemHardware)

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC std::vector<hardware_interface::StateInterface> export_state_interfaces()
        override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC std::vector<hardware_interface::CommandInterface> export_command_interfaces()
        override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC hardware_interface::return_type read(const rclcpp::Time& time,
                                                                           const rclcpp::Duration& period) override;

    ROS2_CONTROL_BASE_MOVEMENT_PUBLIC hardware_interface::return_type write(const rclcpp::Time& time,
                                                                            const rclcpp::Duration& period) override;

   private:
    std::vector<double> _hw_position_commands;
    std::vector<double> _hw_position_states;

    std::vector<double> _hw_velocity_commands;
    std::vector<double> _hw_velocity_states;

    std::string _LOGGER = "BaseMovementSystemHardware";

    std::string _odom_topic;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber_odom;

    rclcpp::Node::SharedPtr _node;

    nav_msgs::msg::Odometry _latest_odometry_msg;

    bool _is_trajectory_execution_in_motion;
    bool _trigger_trajectory_execution;

    geometry_msgs::msg::Point _initial_position;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    float compute_prismatic_joint_state(const geometry_msgs::msg::Point current_odom_positon);
  };

}   // namespace ros2_control_base_movement

#endif   // ROS2_CONTROL_BASE_MOVEMENT_HPP_
