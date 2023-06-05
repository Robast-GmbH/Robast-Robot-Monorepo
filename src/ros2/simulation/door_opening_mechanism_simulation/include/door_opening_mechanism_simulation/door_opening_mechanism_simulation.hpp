#ifndef RB_THERON__DOOR_OPENING_MECHANISM_SIMULATION_HPP_
#define RB_THERON__DOOR_OPENING_MECHANISM_SIMULATION_HPP_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <boost/asio.hpp>   // Used for the async timer
#include <boost/bind.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <memory>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"

namespace door_opening_mechanism_simulation
{

  class DoorMechanismSimulation : public rclcpp::Node
  {
   public:
    DoorMechanismSimulation();
    ~DoorMechanismSimulation(){};

   private:
    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr door_handle_position_subscription_;

    const std::string door_handle_position_topic_ = "/stereo/door_handle_position";

    std::string moveit2_planning_group_name_;
    const std::string default_moveit2_planning_group_name_ = "drawer_planning_group";

    std::shared_ptr<rclcpp::Node> get_shared_pointer_of_node();

    void door_handle_position_callback(const depthai_ros_msgs::msg::SpatialDetectionArray &msg);

    void move_robot_in_simulation_to_target_pose(geometry_msgs::msg::PoseStamped target_pose);

    void open_door_in_simulation(const std::shared_ptr<depthai_ros_msgs::msg::SpatialDetectionArray> door_handle_poses);

    geometry_msgs::msg::PoseStamped convert_pose_to_target_reference_frame(
        const geometry_msgs::msg::PoseStamped pose_in_source_frame, const std::string target_frame);
  };
}   // namespace door_opening_mechanism_simulation
#endif   // RB_THERON__DOOR_OPENING_MECHANISM_SIMULATION_HPP_
