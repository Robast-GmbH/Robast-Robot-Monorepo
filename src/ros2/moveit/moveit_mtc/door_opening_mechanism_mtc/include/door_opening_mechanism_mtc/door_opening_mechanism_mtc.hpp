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

#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"

namespace door_opening_mechanism_mtc
{
  namespace mtc = moveit::task_constructor;

  class DoorMechanismMtc : public rclcpp::Node
  {
   public:
    DoorMechanismMtc();

   private:
    const rclcpp::Logger _LOGGER = rclcpp::get_logger("dom_mtc");

    const std::string _DOOR_HANDLE_POSITION_TOPIC = "/stereo/door_handle_position";

    std::string _planning_group_name;
    const std::string _DEFAULT_PLANNING_GROUP_NAME = "mobile_base_arm";

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr _door_handle_position_subscription;
    mtc::Task _task;

    void handle_node_parameter();

    void create_subscriptions();

    mtc::Task create_task(geometry_msgs::msg::PoseStamped target_pose);

    void do_task(geometry_msgs::msg::PoseStamped target_pose);

    void setup_planning_scene();

    geometry_msgs::msg::PoseStamped convert_pose_to_target_reference_frame(
      const geometry_msgs::msg::PoseStamped pose_in_source_frame, const std::string target_frame);

    void door_handle_position_callback(const depthai_ros_msgs::msg::SpatialDetectionArray& msg);

    void open_door_in_simulation(const std::shared_ptr<depthai_ros_msgs::msg::SpatialDetectionArray> door_handle_poses);
  };
}   // namespace door_opening_mechanism_mtc

#endif   // MOVEIT_MTC_DOOR_OPENING_MECHANISM_MTC_HPP
