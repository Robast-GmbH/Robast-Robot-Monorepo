#ifndef SELF_MAPPING__NAV_ACTION_HPP_
#define SELF_MAPPING__NAV_ACTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_set>
#include <boost/functional/hash.hpp>

class NavAction : public rclcpp::Node
{
public:
  NavAction();
  void navigate_to_frontier();
  
private:
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void find_new_frontiers();
  void send_to_a_frontier();
  void update_current_position();  
  void publish_frontier_markers();
  void send_goal(const geometry_msgs::msg::PoseStamped &goal);
  void handle_goal_result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result, const geometry_msgs::msg::PoseStamped &goal);
  nav2_msgs::action::NavigateToPose::Goal create_goal_message(const geometry_msgs::msg::PoseStamped &goal);
  geometry_msgs::msg::PoseStamped choose_closest_point();
  geometry_msgs::msg::PoseStamped index_to_pose(int index, int width, float resolution);
  bool is_cell_near_obstacle(int index, int width, int height, int occupied_threshold);
  bool is_free_cell(int index,int free_threshold_min, int free_threshold_max);
  bool is_point_close(const geometry_msgs::msg::PoseStamped &point, float threshold);
  bool is_in_blacklist(const geometry_msgs::msg::PoseStamped &point, const std::unordered_set<std::pair<float, float>, boost::hash<std::pair<float, float>>> &blacklist_set);

  std::string _map_frame;
  std::string _base_frame;
  std::string _costmap_topic;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr _action_client;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _costmap_sub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_pub;
  tf2_ros::Buffer _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  nav_msgs::msg::OccupancyGrid _costmap;
  std::vector<geometry_msgs::msg::PoseStamped> _frontiers;
  std::vector<geometry_msgs::msg::PoseStamped> _blacklist;
  geometry_msgs::msg::PoseStamped _current_position;
  bool _is_navigating;
};

#endif  // SELF_MAPPING__NAV_ACTION_HPP_