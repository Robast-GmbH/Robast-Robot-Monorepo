#include "bt_plugins/condition/bool_topic_condition.hpp"

namespace statemachine
{
    BoolTopicCondition::BoolTopicCondition(
        const std::string &name,
        const BT::NodeConfig &config) : BaseCompareCondition(name, config, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort())
    {
        // Nothing special in the default constructor
        initialize_target_value();
    }

    BT::NodeStatus BoolTopicCondition::tick()
    {
        // Please mind we need to call spin_some() TWICE because we have a topic history of 2,
        // so we need to wait for the second message to arrive before we can compare
        callback_group_executor_.spin_some();
        callback_group_executor_.spin_some();
        if (_compare_result)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("BoolTopicCondition"), "BT::NodeStatus::SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_DEBUG(rclcpp::get_logger("BoolTopicCondition"), "BT::NodeStatus::FAILURE");
            return BT::NodeStatus::FAILURE;
        }
    }

    bool BoolTopicCondition::comparator(
        std_msgs::msg::Bool last_message_,
        bool target_value_)
    {
        if ((last_message_.data == target_value_))
        {
            RCLCPP_DEBUG(rclcpp::get_logger("BoolTopicCondition"), "value received: %d", last_message_.data);
            RCLCPP_DEBUG(rclcpp::get_logger("BoolTopicCondition"), "target value should be %d", target_value_);
            return true;
        }
        else
        {
            return false;
        }
    }

    void BoolTopicCondition::callbackTopicFeedback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("BoolTopicCondition"), "callback got called ");
        last_message_ = *msg;
        _compare_result = comparator(last_message_, target_value_);
        new_value_received_ = true;
    }

    void BoolTopicCondition::initialize_target_value()
    {
        getInput("target_value", target_value_);
    }
} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<statemachine::BoolTopicCondition>("BoolTopicCondition");
}