#ifndef MOVEIT_MTC_DOOR_OPENING_MECHANISM_MTC_HPP
#define MOVEIT_MTC_DOOR_OPENING_MECHANISM_MTC_HPP

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit_msgs/msg/workspace_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace door_opening_mechanism_mtc
{
  namespace mtc = moveit::task_constructor;

  class DoorMechanismMtc : public rclcpp::Node
  {
   public:
    DoorMechanismMtc();

   private:
    const rclcpp::Logger _LOGGER = rclcpp::get_logger("dom_mtc");

    std::string _planning_group_name;
    std::string _planning_pipeline;

    std::string _topic_name_pose_stamped;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _door_handle_pose_subscription;
    mtc::Task _task;

    void handle_node_parameter();

    void create_subscriptions();

    mtc::Task create_task(geometry_msgs::msg::PoseStamped target_pose);

    void do_task(const geometry_msgs::msg::PoseStamped target_pose);

    void setup_planning_scene();

    void door_handle_pose_callback(const geometry_msgs::msg::PoseStamped& msg);

    void open_door(const geometry_msgs::msg::PoseStamped door_handle_pose);
  };
}   // namespace door_opening_mechanism_mtc

#endif   // MOVEIT_MTC_DOOR_OPENING_MECHANISM_MTC_HPP
