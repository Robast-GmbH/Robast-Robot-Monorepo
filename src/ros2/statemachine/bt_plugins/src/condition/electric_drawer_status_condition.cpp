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
        communication_interfaces::msg::DrawerStatus last_message_,
        communication_interfaces::msg::DrawerStatus target_value_)
    {
        if (last_message_.drawer_is_open == target_value_.drawer_is_open && new_value_received_)
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

    void ElectricDrawerStatusCondition::callbackDrawerFeedback(const communication_interfaces::msg::DrawerStatus::SharedPtr msg)
    {
        last_message_ = *msg;
        new_value_received_ = true;
    }

    void ElectricDrawerStatusCondition::initialize_target_value()
    {
        getInput("target_value", target_value_.drawer_is_open);
    }
} // namespace drawer_statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::ElectricDrawerStatusCondition>("ElectricDrawerStatusCondition");
}