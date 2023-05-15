#ifndef RB_THERON__DRAWER_BRIDGE_SIMULATION_HPP_
#define RB_THERON__DRAWER_BRIDGE_SIMULATION_HPP_

#include <moveit/move_group_interface/move_group_interface.h>

#include <boost/asio.hpp>   // Used for the async timer
#include <boost/bind.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "communication_interfaces/msg/drawer.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"

namespace drawer_bridge_simulation
{

  class DrawerSimulation : public rclcpp::Node
  {
   public:
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using DrawerLeds = communication_interfaces::msg::DrawerLeds;
    using DrawerStatus = communication_interfaces::msg::DrawerStatus;

    DrawerSimulation();
    ~DrawerSimulation(){};

   private:
    rclcpp::Subscription<DrawerAddress>::SharedPtr open_drawer_subscription_;
    rclcpp::Subscription<DrawerLeds>::SharedPtr drawer_leds_subscription_;
    rclcpp::Publisher<DrawerStatus>::SharedPtr drawer_status_publisher_;

    const float target_pose_open_drawer_ = 0.34;
    const float target_pose_closed_drawer_ = 0;

    uint32_t time_until_drawer_closes_automatically_;
    const int default_time_until_drawer_closes_automatically_ = 3000;   // ms

    std::string moveit2_planning_group_name_;
    const std::string default_moveit2_planning_group_name_ = "drawer_planning_group";

    std::shared_ptr<rclcpp::Node> get_shared_pointer_of_node();

    void open_drawer_topic_callback(const DrawerAddress &msg);

    void move_drawer_in_simulation_to_target_pose(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
        const DrawerAddress drawer_address,
        const float target_pose);

    void async_wait_until_closing_drawer_in_simulation(
        const int time_until_drawer_closing,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
        const DrawerAddress drawer_address,
        const float target_pose);

    void open_drawer_in_simulation(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
                                   const DrawerAddress drawer_address,
                                   const float target_pose);

    void drawer_leds_topic_callback(const DrawerLeds &msg);

    void send_drawer_feedback(communication_interfaces::msg::DrawerStatus drawer_status_msg, bool drawer_is_open);
  };
}   // namespace drawer_bridge_simulation
#endif   // RB_THERON__DRAWER_BRIDGE_SIMULATION_HPP_
