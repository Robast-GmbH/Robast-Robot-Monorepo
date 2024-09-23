#ifndef SELF_MAPPING__NAV_ACTION_HPP_
#define SELF_MAPPING__NAV_ACTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class NavAction : public rclcpp::Node
{
public:
  NavAction();

  void navigate_to_frontier();
  
private:
  void send_goal(const geometry_msgs::msg::PoseStamped &goal);
  geometry_msgs::msg::PoseStamped choose_closest_point();
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void find_new_frontiers();
  void send_to_a_frontier();
  void update_current_position();  
  void publish_frontier_markers();
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::OccupancyGrid costmap_;
  std::vector<geometry_msgs::msg::PoseStamped> frontiers_;
  std::vector<geometry_msgs::msg::PoseStamped> blacklist_;
  geometry_msgs::msg::PoseStamped current_position_;
  bool is_navigating_;
};

#endif  // SELF_MAPPING__NAV_ACTION_HPP_