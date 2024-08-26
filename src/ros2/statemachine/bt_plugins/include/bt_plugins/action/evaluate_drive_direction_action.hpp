#ifndef STATEMACHINE__BT_PLUGINS__ACTION__EVALUATEDRIVEDIRECTION_BT_NODES_H
#define STATEMACHINE__BT_PLUGINS__ACTION__EVALUATEDRIVEDIRECTION_BT_NODES_H

#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "bt_plugins/utils/calculate_direction.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace statemachine
{
  /**
   * @brief A BT::ConditionNode that returns SUCCESS when goal is
   * updated on the blackboard and FAILURE otherwise
   */
  class EvaluateDriveDirection : public BT::SyncActionNode
  {
  public:
    EvaluateDriveDirection(
        const std::string &name,
        const BT::NodeConfig &config);

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
      return {
          BT::InputPort<std::string>("path_topic", "topic"),
          BT::InputPort<uint16_t>("prediction_horizon", "prediction_horizon"),
          BT::OutputPort<std::string>("direction", "standing")};
    }

  private:
    rclcpp::Node::SharedPtr _node;

    std::string _topic_name = "plan";
    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _drawer_open_sub;
    nav_msgs::msg::Path _path = nav_msgs::msg::Path();
    std::string _direction;
    geometry_msgs::msg::PoseStamped _global_pose;
    int _current_path_index = 0;
    std::shared_ptr<tf2_ros::Buffer> _tf;
    std::shared_ptr<tf2_ros::TransformListener> _transform_listener;
    uint16_t _prediction_horizon = 60;

    void exposeDriveDirection();
    void callbackPathReceived(const nav_msgs::msg::Path::SharedPtr msg);
    int getCurrentIndex(const geometry_msgs::msg::Pose &current_pose, const nav_msgs::msg::Path &path);
  };
} // namespace statemachine
#endif