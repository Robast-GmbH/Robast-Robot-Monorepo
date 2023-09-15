#ifndef DRAWER_STATEMACHINE_LED_CHANGED_CONDITION_HPP_
#define DRAWER_STATEMACHINE_LED_CHANGED_CONDITION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "bt_base_types/LED_base.hpp"

namespace statemachine
{
    class LEDChangedCondition : public BT::ConditionNode
    {
    public:
        LEDChangedCondition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<bt_base_types::LED>>("led_vector", "{leds}")};
        }
        BT::NodeStatus tick() override;

    private:
        std::vector<bt_base_types::LED> _old_led_vector;
    };
} // namespace statemachine

#endif // DRAWER_STATEMACHINE_LED_CHANGED_CONDITION_HPP_
