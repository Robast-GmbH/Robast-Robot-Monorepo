#ifndef STATEMACHINE__BT_PLUGINS__ACTION__EVALUATEDRIVEDIRECTION_BT_NODES_H
#define STATEMACHINE__BT_PLUGINS__ACTION__EVALUATEDRIVEDIRECTION_BT_NODES_H

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "bt_plugins/utils/calculate_direction.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace statemachine
{
  constexpr uint8_t STANDING_THRESHOLD_IN_S = 1;
  constexpr uint8_t SLEEPING_THRESHOLD_IN_S = 10;

  /**
   * @brief A BT::ConditionNode that returns SUCCESS when goal is
   * updated on the blackboard and FAILURE otherwise
   */
  class EvaluateDriveDirection : public BT::SyncActionNode
  {
   public:
    EvaluateDriveDirection(const std::string &name, const BT::NodeConfig &config);

    EvaluateDriveDirection() = delete;

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;
    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("global_path_topic", "plan"),
              BT::InputPort<std::string>("local_path_topic", "local_plan"),
              BT::InputPort<uint16_t>("prediction_horizon", "prediction_horizon"),
              BT::InputPort<std::string>("global_frame", "map"),
              BT::InputPort<std::string>("base_frame", "base_link"),
              BT::OutputPort<std::string>("direction", "standing")};
    }

   private:
    rclcpp::Node::SharedPtr _node;

    std::string _global_path_topic_name = "plan";
    std::string _local_path_topic_name = "local_plan";

    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _global_path_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _local_path_sub;

    nav_msgs::msg::Path _global_path = nav_msgs::msg::Path();

    builtin_interfaces::msg::Time _timestamp_last_local_path;

    std::string _direction;
    geometry_msgs::msg::PoseStamped _global_pose;
    int _current_path_index = 0;
    std::shared_ptr<tf2_ros::Buffer> _tf;
    std::shared_ptr<tf2_ros::TransformListener> _transform_listener;
    uint16_t _prediction_horizon = 60;
    std::string _global_frame = "map";
    std::string _base_frame = "base_link";

    void exposeDriveDirection();
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    int getCurrentIndex(const geometry_msgs::msg::Pose &current_pose, const nav_msgs::msg::Path &path);

    bool is_robot_standing(const builtin_interfaces::msg::Time current_time);
    bool is_robot_sleeping(const builtin_interfaces::msg::Time current_time);
  };
}   // namespace statemachine
#endif