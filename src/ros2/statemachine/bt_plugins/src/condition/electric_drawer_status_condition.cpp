#include "bt_plugins/condition/electric_drawer_status_condition.hpp"

namespace drawer_statemachine
{
    ElectricDrawerStatusCondition::ElectricDrawerStatusCondition(
        const std::string &name,
        const BT::NodeConfig &config) : BaseCompareCondition(name, config)
    {
        // Nothing special in the default constructor
        initialize_target_value();
    }

    BT::NodeStatus ElectricDrawerStatusCondition::tick()
    {
        callback_group_executor_.spin_some();
        RCLCPP_DEBUG(rclcpp::get_logger("BaseCompareCondition"), "ticked");
        if (comparator(last_message_, target_value_))
        {
            RCLCPP_DEBUG(rclcpp::get_logger("BaseCompareCondition"), "BT::NodeStatus::SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }
        else if (std::chrono::steady_clock::now() -
                     blackboard_->get<std::chrono::steady_clock::time_point>("transition_time") >=
                 timeout_duration_)
        {
            RCLCPP_WARN(rclcpp::get_logger("BaseCompareCondition"), "timeout compare condition");
            // Timeout
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    bool ElectricDrawerStatusCondition::comparator(
        communication_interfaces::msg::ElectricalDrawerStatus last_message_,
        uint8_t target_value_)
    {
        if (((target_value_ > 250 || last_message_.position <= target_value_ + 5) && (target_value_ < 5 || last_message_.position >= target_value_ - 5)) && new_value_received_)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("ElectricDrawerStatusCondition"), "value received: %d", last_message_.position);
            RCLCPP_DEBUG(rclcpp::get_logger("ElectricDrawerStatusCondition"), "target value should be %d", target_value_);
            new_value_received_ = false;
            return true;
        }
        else if (new_value_received_)
        {
            new_value_received_ = false;
            return false;
        }
        else
        {
            new_value_received_ = false;
            return false;
        }
    }

    void ElectricDrawerStatusCondition::callbackDrawerFeedback(const communication_interfaces::msg::ElectricalDrawerStatus::SharedPtr msg)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("ElectricDrawerStatusCondition"), "callback got called ");
        last_message_ = *msg;
        new_value_received_ = true;
    }

    void ElectricDrawerStatusCondition::initialize_target_value()
    {
        getInput("target_value", target_value_);
    }
} // namespace drawer_statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::ElectricDrawerStatusCondition>("ElectricDrawerStatusCondition");
}