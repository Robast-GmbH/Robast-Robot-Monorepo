#include "bt_plugins/condition/drawer_status_condition.hpp"

namespace statemachine
{
    DrawerStatusCondition::DrawerStatusCondition(
        const std::string &name,
        const BT::NodeConfig &config) : BT::ConditionNode(name, config)
    {
        _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        _callback_group = _node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

        getInput("topic", _topic_name);

        if (_topic_name == "")
        {
            auto var = getInput<std::string>("topic");
            _topic_name = "/drawer_is_open";
        }

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = _callback_group;
        _drawer_status_sub = _node->create_subscription<communication_interfaces::msg::DrawerStatus>(
            _topic_name,
            qos,
            std::bind(&DrawerStatusCondition::callbackDrawerFeedback, this, std::placeholders::_1),
            sub_option);

        getInput("target_value", _target_value);
        _last_message = !_target_value;
    }

    BT::NodeStatus DrawerStatusCondition::tick()
    {
        _callback_group_executor.spin_some();

        if (_last_message == _target_value)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void DrawerStatusCondition::callbackDrawerFeedback(const communication_interfaces::msg::DrawerStatus::SharedPtr msg)
    {
        _last_message = msg->drawer_is_open;
    }
} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<statemachine::DrawerStatusCondition>("DrawerStatusCondition");
}