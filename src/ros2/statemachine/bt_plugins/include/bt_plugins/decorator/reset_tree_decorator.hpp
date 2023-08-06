#ifndef BT_PLUGINS__BT_BASE_ERROR_DECORATOR_HPP
#define BT_PLUGINS__BT_BASE_ERROR_DECORATOR_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp/decorator_node.h"

namespace statemachine
{
  class ResetDecorator : public BT::DecoratorNode
  {

  public:
    ResetDecorator(const std::string &name, const BT::NodeConfig &config);
    BT::NodeStatus tick();
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("topic", "/reset")};
    }

  protected:
    void callbackDrawerFeedback(const std_msgs::msg::Bool::SharedPtr msg);
    void logError();
    typename rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drawer_status_sub_;
    std::string topic_name_;
    BT::Blackboard::Ptr blackboard_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::CallbackGroup::SharedPtr _callback_group;
  };
} // namespace statemachine
#endif // BT_PLUGINS__BT_BASE_ERROR_DECORATOR_HPP