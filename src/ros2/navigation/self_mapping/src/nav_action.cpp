#include <random>
#include <cmath>
#include "self_mapping/nav_action.hpp"

NavAction::NavAction() : Node("self_mapping_node"), _tf_buffer(get_clock()), _is_navigating(false)
{
  //parameters
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("costmap_topic", "global_costmap/costmap");
  this->declare_parameter<std::string>("base_frame", "base_link");

  this->get_parameter("map_frame", _map_frame);
  this->get_parameter("costmap_topic", _costmap_topic);
  this->get_parameter("base_frame", _base_frame);
  _action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

  rclcpp::QoS _costmap_qos_profile(1);  
  _costmap_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  _costmap_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); 

  _costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      _costmap_topic, _costmap_qos_profile, std::bind(&NavAction::costmap_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "NavAction Node Initialized");
  
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(_tf_buffer);
  _marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
}

nav2_msgs::action::NavigateToPose::Goal NavAction::create_goal_message(const geometry_msgs::msg::PoseStamped &goal)
{
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose.pose.position = goal.pose.position;
  goal_msg.pose.pose.orientation.w = 1.;
  goal_msg.pose.header.frame_id = _map_frame;
  goal_msg.pose.header.stamp = this->now();
  goal_msg.behavior_tree = "/workspace/src/navigation/nav_bringup/behavior_trees/humble/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml";
  return goal_msg;
}

void NavAction::send_goal(const geometry_msgs::msg::PoseStamped &goal)
{
  if (!_action_client->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available!");
    return ;
  }

  auto goal_msg = create_goal_message(goal);

  RCLCPP_DEBUG(this->get_logger(), "Sending goal to point: (%f, %f)", goal.pose.position.x, goal.pose.position.y);
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback = [this, goal](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    handle_goal_result(result, goal);
    update_current_position();
    _is_navigating = false;
    if (!_frontiers.empty()) {
            RCLCPP_INFO(this->get_logger(), "Sending to a frontier...");
            auto next_goal = choose_closest_point();
            send_goal(next_goal);
        } else {
            RCLCPP_INFO(this->get_logger(), "Frontier list is empty. Waiting for new frontiers...");
        }

    };

    _action_client->async_send_goal(goal_msg, send_goal_options);
    _is_navigating = true;
}

void NavAction::handle_goal_result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result, const geometry_msgs::msg::PoseStamped &goal)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_WARN(this->get_logger(), "Goal was aborted");
    _blacklist.push_back(goal);
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "Goal was canceled");
    _blacklist.push_back(goal);
  }
}

void NavAction::navigate_to_frontier()
{
  if (_frontiers.empty()) {
    RCLCPP_INFO(this->get_logger(), "No frontiers found. Waiting for new costmap...");
  } else {
    send_to_a_frontier();
  }
} 

void NavAction::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Costmap callback triggered");
  _costmap = *msg;
  if (_frontiers.empty()) {
    find_new_frontiers();
  }
  else {
    send_to_a_frontier();
  }
}

void NavAction::find_new_frontiers()  
{  
  RCLCPP_INFO(this->get_logger(), "Finding new frontiers...");  
  _frontiers.clear();  
  int width = _costmap.info.width;  
  int height = _costmap.info.height;  
  float resolution = _costmap.info.resolution;  
  int free_threshold_min = 0;    
  int free_threshold_max = 50; 
  int occupied_threshold = 80;  

  // Convert blacklist to a set for faster lookups  
  std::unordered_set<std::pair<float, float>, boost::hash<std::pair<float, float>>> blacklist_set;  
  for (const auto &blacklisted_point : _blacklist) {  
    blacklist_set.emplace(blacklisted_point.pose.position.x, blacklisted_point.pose.position.y);  
  }  

  for (int i = 0; i < width * height; ++i) {  
    if (is_free_cell(i, free_threshold_min, free_threshold_max) && !is_cell_near_obstacle(i, width, height, occupied_threshold)) {  
      auto point = index_to_pose(i, width, resolution);  
      if (!is_in_blacklist(point, blacklist_set)) {  
        _frontiers.push_back(point);  
      }  
    }  
  }  

  RCLCPP_DEBUG(this->get_logger(), "Number of frontiers found: %zu", _frontiers.size());  
}  

geometry_msgs::msg::PoseStamped NavAction::index_to_pose(int index, int width, float resolution)  
{  
  geometry_msgs::msg::PoseStamped point;  
  point.pose.position.x = (index % width) * resolution + _costmap.info.origin.position.x;  
  point.pose.position.y = (index / width) * resolution + _costmap.info.origin.position.y;  
  return point;  
}  

bool NavAction::is_in_blacklist(const geometry_msgs::msg::PoseStamped &point, const std::unordered_set<std::pair<float, float>, boost::hash<std::pair<float, float>>> &blacklist_set)  
{  
  return blacklist_set.find({point.pose.position.x, point.pose.position.y}) != blacklist_set.end();  
} 

bool NavAction::is_cell_near_obstacle(int index, int width, int height, int occupied_threshold)
{
  int x = index % width;  
  int y = index / width;  
  for (int dx = -1; dx <= 1; ++dx) {  
    for (int dy = -1; dy <= 1; ++dy) {  
      int neighbor_x = x + dx;  
      int neighbor_y = y + dy;  
      if (neighbor_x >= 0 && neighbor_x < width && neighbor_y >= 0 && neighbor_y < height) {  
        int neighbor_idx = neighbor_x + neighbor_y * width;  
        if (_costmap.data[neighbor_idx] >= occupied_threshold) {  
          return true;  
        }  
      }  
    }  
  }  
  return false;  
}

bool NavAction::is_free_cell(int index, int free_threshold_min, int free_threshold_max)
{
  return _costmap.data[index] >= free_threshold_min && _costmap.data[index] <= free_threshold_max;
}

void NavAction::send_to_a_frontier(){
  if (!_frontiers.empty() && !_is_navigating){
    RCLCPP_INFO(this->get_logger(), "Sending to a frontier...");
    auto goal = choose_closest_point();
    send_goal(goal);
  }
  else if (_is_navigating) {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for current navigation to finish...");
  }
  else if (_frontiers.empty()) {
    RCLCPP_INFO(this->get_logger(), "No frontiers found. Waiting for new costmap...");
  }
}

geometry_msgs::msg::PoseStamped NavAction::choose_closest_point()
{
  geometry_msgs::msg::PoseStamped closest_point;
  float min_distance = std::numeric_limits<float>::max();
  float significant_distance_threshold = 4.0;
  bool found_valid_goal = false;

  for (const auto &point : _frontiers) {
    float distance = std::hypot(point.pose.position.x - _current_position.pose.position.x,
                                point.pose.position.y - _current_position.pose.position.y);

    if (distance < significant_distance_threshold) {
      RCLCPP_DEBUG(this->get_logger(), "Skipping close frontier at (%f, %f) - Distance: %f", 
                  point.pose.position.x, point.pose.position.y, distance);
      continue;
    }  

    if (distance < min_distance) {
      min_distance = distance;
      closest_point = point;
      found_valid_goal = true;
    }
  }

  if (!found_valid_goal) {
    RCLCPP_WARN(this->get_logger(), "No frontier found that is far enough from the current position.");
  }

  return closest_point;
}

void NavAction::update_current_position()
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped = _tf_buffer.lookupTransform(_map_frame, _base_frame, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform map frame to base frame: %s", ex.what());
    return;
  }

  _current_position.pose.position.x = transform_stamped.transform.translation.x;
  _current_position.pose.position.y = transform_stamped.transform.translation.y;
  _current_position.pose.position.z = transform_stamped.transform.translation.z;

  _current_position.pose.orientation = transform_stamped.transform.rotation;

  RCLCPP_DEBUG(this->get_logger(), "Current Position updated from transform: x: %f, y: %f", 
              _current_position.pose.position.x, _current_position.pose.position.y);
}

void NavAction::publish_frontier_markers()
{
  visualization_msgs::msg::MarkerArray _marker_array;
  
  int id = 0; 

  for (const auto &frontier : _frontiers) {
    visualization_msgs::msg::Marker _marker;

    _marker.header.frame_id = _map_frame;  
    _marker.header.stamp = this->get_clock()->now();
    _marker.ns = "frontiers";
    _marker.id = id++;  
    _marker.type = visualization_msgs::msg::Marker::SPHERE;  
    _marker.action = visualization_msgs::msg::Marker::ADD;

    _marker.pose.position.x = frontier.pose.position.x;
    _marker.pose.position.y = frontier.pose.position.y;
    _marker.pose.position.z = 0.0; 

    _marker.scale.x = 0.2;
    _marker.scale.y = 0.2;
    _marker.scale.z = 0.2;

    _marker.color.r = 0.0f;
    _marker.color.g = 1.0f;
    _marker.color.b = 0.0f;
    _marker.color.a = 1.0;
    
    _marker.lifetime = rclcpp::Duration::from_seconds(0);  

    _marker_array.markers.push_back(_marker);
  }

  // Publish the markers
  _marker_pub->publish(_marker_array);

  RCLCPP_DEBUG(this->get_logger(), "Published %zu frontier markers.", _marker_array.markers.size());
}
