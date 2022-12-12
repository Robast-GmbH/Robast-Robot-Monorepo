#include "bt_plugins/condition/drawer_status_condition.hpp"


namespace drawer_statemachine
{
    DrawerStatusCondition::DrawerStatusCondition(
        const std::string& name,
        const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        getInput("topic", topic_name_);

        if (topic_name_ == "")
        {
            auto var = getInput<std::string>("topic");
            std::cout << var.value() << std::endl;
            topic_name_ = "/drawer_is_open";
        }

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        drawer_status_sub_ = node_->create_subscription<communication_interfaces::msg::DrawerStatus>(
            topic_name_,
            qos,
            std::bind(&DrawerStatusCondition::callbackDrawerFeedback, this, std::placeholders::_1),
            sub_option);
        last_message_ = false;

        getInput("target_value", target_value_);
    }

    BT::NodeStatus DrawerStatusCondition::tick()
    {
        callback_group_executor_.spin_some();

        if (last_message_ == target_value_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void DrawerStatusCondition::callbackDrawerFeedback(const communication_interfaces::msg::DrawerStatus::SharedPtr msg)
    {
        last_message_ = msg->drawer_is_open;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::DrawerStatusCondition>("DrawerStatusCondition");
}