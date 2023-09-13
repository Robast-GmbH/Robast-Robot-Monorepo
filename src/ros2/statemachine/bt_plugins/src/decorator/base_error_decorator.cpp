#include "bt_plugins/decorator/base_error_decorator.hpp"
#include "error_utils/error_definitions.hpp"

namespace statemachine
{
  BaseErrorDecorator::BaseErrorDecorator(const std::string &name, const BT::NodeConfig &config) : BT::DecoratorNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    _callback_group = _node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
    callback_group_executor_.add_callback_group(_callback_group, _node->get_node_base_interface());

    getInput("topic", topic_name_);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.transient_local().best_effort();

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    drawer_status_sub_ = _node->create_subscription<communication_interfaces::msg::ErrorBaseMsg>(
        topic_name_,
        qos,
        std::bind(&BaseErrorDecorator::
                      callbackDrawerFeedback,
                  this, std::placeholders::_1),
        sub_option);
    blackboard_ = config.blackboard;
  }

  void BaseErrorDecorator::logError()
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("BaseErrorDecorator"), "Recieved an error: " << _error_msg->error_description);
  }

  BT::NodeStatus BaseErrorDecorator::tick()
  {
    BT::NodeStatus prev_status = status();
    if (prev_status == BT::NodeStatus::IDLE)
    {
      setStatus(BT::NodeStatus::RUNNING);
    }
    callback_group_executor_.spin_some();
    if (_is_error_received)
    {
      _is_error_received = false;
      resetChild();
      return BT::NodeStatus::FAILURE;
    }
    const BT::NodeStatus child_status = child_node_->executeTick();
    if (isStatusCompleted(child_status))
    {
      resetChild();
    }
    return child_status;
  }

  void BaseErrorDecorator::callbackDrawerFeedback(const communication_interfaces::msg::ErrorBaseMsg::SharedPtr msg)
  {
    switch (msg->error_code)
    {
    case ERROR_CODES_TIMEOUT_DRAWER_NOT_OPENED:
      // TODO @robast: there is some weird bug with the data formatation of the serialized error_data. The string has a random cutoff
      //  if (converter_.stringToMessage(msg->error_data) == blackboard_->get<communication_interfaces::msg::DrawerAddress>("drawer_address"))
      //  {
      _error_msg = msg;
      _is_error_received = true;
      logError();
      // }
      break;

    default:
      break;
    }
  }
} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::BaseErrorDecorator>("BaseErrorDecorator");
}