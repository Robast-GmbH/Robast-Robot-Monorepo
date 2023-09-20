#ifndef BT_PLUGINS__BT_BASE_ERROR_DECORATOR_HPP
#define BT_PLUGINS__BT_BASE_ERROR_DECORATOR_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/decorator_node.h"

#include "error_utils/generic_error_converter.hpp"
#include "communication_interfaces/msg/error_base_msg.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace statemachine
{
  class BaseErrorDecorator : public BT::DecoratorNode
  {

  public:
    BaseErrorDecorator(const std::string &name, const BT::NodeConfig &config);
    BT::NodeStatus tick();
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("topic", "/robast_error")};
    }

  protected:
    void callbackDrawerFeedback(const communication_interfaces::msg::ErrorBaseMsg::SharedPtr msg);
    void logError();
    typename rclcpp::Subscription<communication_interfaces::msg::ErrorBaseMsg>::SharedPtr drawer_status_sub_;
    std::string topic_name_;
    BT::Blackboard::Ptr blackboard_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    MessageConverter<communication_interfaces::msg::DrawerAddress> converter_;

  private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::CallbackGroup::SharedPtr _callback_group;
    communication_interfaces::msg::ErrorBaseMsg::SharedPtr _error_msg;
    bool _is_error_received = false;
  };
} // namespace statemachine
#endif // BT_PLUGINS__BT_BASE_ERROR_DECORATOR_HPP