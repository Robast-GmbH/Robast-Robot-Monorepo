#ifndef MOVEIT_MTC_DOOR_OPENING_MECHANISM_MTC_HPP
#define MOVEIT_MTC_DOOR_OPENING_MECHANISM_MTC_HPP

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit_msgs/msg/workspace_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace door_opening_mechanism_mtc
{
  namespace mtc = moveit::task_constructor;

  constexpr float DISTANCE_TO_PUSH_DOOR_HANDLE_DOWN = 0.055;
  constexpr float FORWARD_DISTANCE_TO_PUSH_DOOR_OUT_OF_LATCH = 0.05;
  constexpr float FORWARD_DISTANCE_TO_PUSH_DOOR_OPEN = 0.4;
  constexpr float DISTANCE_TO_OTHER_SIDE_OF_DOOR = 0.5;
  constexpr float DOOR_DETECTION_OFFSET = 0.077;
  constexpr float Z_DOOR_DETECTION_OFFSET = 0.04;

  class DoorMechanismMtc : public rclcpp::Node
  {
   public:
    DoorMechanismMtc(const rclcpp::NodeOptions& options);

   private:
    const rclcpp::Logger _LOGGER = rclcpp::get_logger("dom_mtc");

    std::string _planning_group_name;
    std::string _planning_pipeline;

    std::string _topic_name_pose_stamped;
    std::string _topic_name_trigger_door_opening;

    double _door_handle_pose_timeout;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _door_handle_pose_subscription;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _trigger_door_opening_subscription;
    mtc::Task _task;

    geometry_msgs::msg::PoseStamped _latest_door_handle_pose;

    void handle_node_parameter();

    void create_subscriptions();

    mtc::Task create_task(geometry_msgs::msg::PoseStamped door_handle_pose);

    void do_task(const geometry_msgs::msg::PoseStamped door_handle_pose);

    void setup_planning_scene();

    void door_handle_pose_callback(const geometry_msgs::msg::PoseStamped& msg);

    void trigger_door_opening_callback(const std_msgs::msg::Empty& msg);

    void open_door();
  };
}   // namespace door_opening_mechanism_mtc

#endif   // MOVEIT_MTC_DOOR_OPENING_MECHANISM_MTC_HPP
