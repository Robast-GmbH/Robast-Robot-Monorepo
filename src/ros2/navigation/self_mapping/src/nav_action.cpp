#include "self_mapping/nav_action.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include <random>
#include <cmath>

NavAction::NavAction() : Node("self_mapping_node"), tf_buffer_(get_clock()), is_navigating_(false)
{
  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

  rclcpp::QoS costmap_qos_profile(1);  
  costmap_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  costmap_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); 

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "global_costmap/costmap", costmap_qos_profile, std::bind(&NavAction::costmap_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(this->get_logger(), "NavAction Node Initialized");
  
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
}

void NavAction::send_goal(const geometry_msgs::msg::PoseStamped &goal)
{
  if (!action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available!");
    return ;
  }

  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose.pose.position = goal.pose.position;
  goal_msg.pose.pose.orientation.w = 1.;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.behavior_tree = "/workspace/src/navigation/nav_bringup/behavior_trees/humble/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml";

  RCLCPP_DEBUG(this->get_logger(), "Sending goal to point: (%f, %f)", goal.pose.position.x, goal.pose.position.y);
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback = [this, goal](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_WARN(this->get_logger(), "Goal was aborted");
        blacklist_.push_back(goal);
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        blacklist_.push_back(goal);
    }
    update_current_position();
    is_navigating_ = false;
    
    if (!frontiers_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Sending to a frontier...");
            auto next_goal = choose_closest_point();
            send_goal(next_goal);
        } else {
            RCLCPP_INFO(this->get_logger(), "Frontier list is empty. Waiting for new frontiers...");
        }

    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
    is_navigating_ = true;
}

void NavAction::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Costmap callback triggered");
  costmap_ = *msg;
  if (frontiers_.empty()) {
    find_new_frontiers();
  }
  else {
    send_to_a_frontier();
  }
}

void NavAction::find_new_frontiers()
{
  RCLCPP_INFO(this->get_logger(), "Finding new frontiers...");
  frontiers_.clear();
  int width = costmap_.info.width;
  int height = costmap_.info.height;
  float resolution = costmap_.info.resolution;
  int free_threshold_min = 0;    
  int free_threshold_max = 50; 
  int occupied_threshold = 80;

  for (int i = 0; i < width * height; ++i) {
    
     if (costmap_.data[i] >= free_threshold_min && costmap_.data[i] <= free_threshold_max) {
      bool is_near_obstacle = false;

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          int neighbor_x = (i % width) + dx;
          int neighbor_y = (i / width) + dy;

          if (neighbor_x >= 0 && neighbor_x < width && neighbor_y >= 0 && neighbor_y < height) {
            int neighbor_idx = neighbor_x + neighbor_y * width;
            if (costmap_.data[neighbor_idx] >= occupied_threshold) {
              is_near_obstacle = true;
              break;
            }
          }
        }
        if (is_near_obstacle) {
          break;
        }
      }

      if (!is_near_obstacle) {
        geometry_msgs::msg::PoseStamped point;
        point.pose.position.x = (i % width) * resolution + costmap_.info.origin.position.x;
        point.pose.position.y = (i / width) * resolution + costmap_.info.origin.position.y;
        bool in_blacklist = false;
        for (const auto &blacklisted_point : blacklist_) {
                    if (blacklisted_point.pose.position.x == point.pose.position.x && 
                        blacklisted_point.pose.position.y == point.pose.position.y) {
                        in_blacklist = true;
                        break;
                    }
        }
        if (!in_blacklist) {
          frontiers_.push_back(point);
        }
      }
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "Number of frontiers found: %zu", frontiers_.size());

}

void NavAction::send_to_a_frontier(){
  if (!frontiers_.empty() && !is_navigating_){
    RCLCPP_INFO(this->get_logger(), "Sending to a frontier...");
    auto goal = choose_closest_point();
    send_goal(goal);
  }
  else if (is_navigating_) {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for current navigation to finish...");
  }
  else if (frontiers_.empty()) {
    RCLCPP_INFO(this->get_logger(), "No frontiers found. Waiting for new costmap...");
  }
}

geometry_msgs::msg::PoseStamped NavAction::choose_closest_point()
{
  geometry_msgs::msg::PoseStamped closest_point;
  float min_distance = std::numeric_limits<float>::max();
  float significant_distance_threshold = 4.0;
  bool found_valid_goal = false;

  for (const auto &point : frontiers_) {
    float distance = std::hypot(point.pose.position.x - current_position_.pose.position.x,
                                point.pose.position.y - current_position_.pose.position.y);

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
    transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform 'map' to 'base_link': %s", ex.what());
    return;
  }

  current_position_.pose.position.x = transform_stamped.transform.translation.x;
  current_position_.pose.position.y = transform_stamped.transform.translation.y;
  current_position_.pose.position.z = transform_stamped.transform.translation.z;

  current_position_.pose.orientation = transform_stamped.transform.rotation;

  RCLCPP_INFO(this->get_logger(), "Current Position updated from transform: x: %f, y: %f", 
              current_position_.pose.position.x, current_position_.pose.position.y);
}

void NavAction::publish_frontier_markers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  int id = 0; 

  for (const auto &frontier : frontiers_) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";  
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "frontiers";
    marker.id = id++;  
    marker.type = visualization_msgs::msg::Marker::SPHERE;  
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = frontier.pose.position.x;
    marker.pose.position.y = frontier.pose.position.y;
    marker.pose.position.z = 0.0; 

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0);  

    marker_array.markers.push_back(marker);
  }

  // Publish the markers
  marker_pub_->publish(marker_array);

  RCLCPP_DEBUG(this->get_logger(), "Published %zu frontier markers.", marker_array.markers.size());
}
