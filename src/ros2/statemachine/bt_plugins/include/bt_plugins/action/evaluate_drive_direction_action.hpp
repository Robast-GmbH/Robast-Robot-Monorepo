#ifndef STATEMACHINE__BT_PLUGINS__ACTION__EVALUATEDRIVEDIRECTION_BT_NODES_H
#define STATEMACHINE__BT_PLUGINS__ACTION__EVALUATEDRIVEDIRECTION_BT_NODES_H

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "bt_plugins/utils/calculate_direction.hpp"

namespace statemachine
{
  /**
   * @brief A BT::ConditionNode that returns SUCCESS when goal is
   * updated on the blackboard and FAILURE otherwise
   */
  class EvaluateDriveDirection : public BT::StatefulActionNode
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
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("path_topic", "topic", "empty"),
          BT::OutputPort<std::string>("direction", "standing", "empty")};
    }

  private:
    rclcpp::Node::SharedPtr _node;

    std::string _topic_name;
    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _drawer_open_sub;
    nav_msgs::msg::Path _path;
    std::string _direction;
    rclcpp::TimerBase::SharedPtr _timer;

    void exposeDriveDirectionTimerCallback();
    void callbackPathReceived(const nav_msgs::msg::Path::SharedPtr msg);
  };
} // namespace statemachine
#endif