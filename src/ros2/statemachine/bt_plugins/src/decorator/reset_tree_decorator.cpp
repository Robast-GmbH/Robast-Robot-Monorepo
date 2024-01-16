#include "bt_plugins/decorator/reset_tree_decorator.hpp"

namespace statemachine
{
  ResetDecorator::ResetDecorator(const std::string &name, const BT::NodeConfig &config) : BT::DecoratorNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    _callback_group = _node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
    callback_group_executor_.add_callback_group(_callback_group, _node->get_node_base_interface());

    getInput("topic", topic_name_);

    if (topic_name_.empty())
    {
      topic_name_ = "/reset";
    }

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.transient_local().best_effort();

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    reset_signal_sub_ = _node->create_subscription<std_msgs::msg::Bool>(
        topic_name_,
        qos,
        std::bind(&ResetDecorator::
                      callbackResetFeedback,
                  this, std::placeholders::_1),
        sub_option);
  }

  BT::NodeStatus ResetDecorator::tick()
  {
    BT::NodeStatus prev_status = status();
    if (prev_status == BT::NodeStatus::IDLE)
    {
      setStatus(BT::NodeStatus::RUNNING);
    }
    callback_group_executor_.spin_some();
    const BT::NodeStatus child_status = child_node_->executeTick();
    if (isStatusCompleted(child_status))
    {
      resetChild();
    }
    return child_status;
  }

  void ResetDecorator::callbackResetFeedback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      resetChild();
    }
  }
} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::ResetDecorator>("ResetDecorator");
}