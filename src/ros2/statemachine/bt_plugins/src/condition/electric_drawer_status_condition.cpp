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

    bool ElectricDrawerStatusCondition::comparator(
        communication_interfaces::msg::ElectricalDrawerStatus last_message_,
        uint8_t target_value_)
    {
        if ((last_message_.position > 250 || last_message_.position <= target_value_ + 5) && (last_message_.position < 5 || last_message_.position >= target_value_ - 5) && new_value_received_)
        {
            new_value_received_ = false;
            return true;
        }
        else if (new_value_received_)
        {
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