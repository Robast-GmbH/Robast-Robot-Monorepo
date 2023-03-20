#ifndef RB_THERON__DOOR_OPENING_MECHANISM_SIMULATION_HPP_
#define RB_THERON__DOOR_OPENING_MECHANISM_SIMULATION_HPP_

// #include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/asio.hpp>   // Used for the async timer
#include <boost/bind.hpp>
#include <iostream>
#include <memory>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "communication_interfaces/msg/drawer.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"

namespace door_opening_mechanism_simulation
{

  class DoorMechanismSimulation : public rclcpp::Node
  {
   public:
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using DrawerLeds = communication_interfaces::msg::DrawerLeds;
    using DrawerStatus = communication_interfaces::msg::DrawerStatus;

    DoorMechanismSimulation();
    ~DoorMechanismSimulation(){};

   private:
    rclcpp::Subscription<DrawerAddress>::SharedPtr open_door_subscription_;

    const float target_pose_open_drawer_ = 0.34;
    const float target_pose_closed_drawer_ = 0;

    std::string moveit2_planning_group_name_;
    const std::string default_moveit2_planning_group_name_ = "drawer_planning_group";

    std::shared_ptr<rclcpp::Node> get_shared_pointer_of_node();

    void open_door_topic_callback(const DrawerAddress &msg);

    void move_robot_in_simulation_to_target_pose(const float target_pose);

    void open_door_in_simulation(const float target_pose);
  };
}   // namespace door_opening_mechanism_simulation
#endif   // RB_THERON__DOOR_OPENING_MECHANISM_SIMULATION_HPP_
