#include "bt_plugins/condition/led_changed_condition.hpp"

namespace statemachine
{
    LEDChangedCondition::LEDChangedCondition(
        const std::string &name,
        const BT::NodeConfig &config) : BT::ConditionNode(name, config)
    {
        _old_led_vector = std::vector<bt_base_types::LED>();
    }

    BT::NodeStatus LEDChangedCondition::tick()
    {
        std::vector<bt_base_types::LED> led_vector;
        getInput("led_vector", led_vector);

        if (led_vector.size() != _old_led_vector.size())
        {
            _old_led_vector = led_vector;
            return BT::NodeStatus::SUCCESS;
        }

        bool are_vectors_equal = std::equal(led_vector.begin(), led_vector.end(), _old_led_vector.begin());
        if (!are_vectors_equal)
        {
            _old_led_vector = led_vector;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<statemachine::LEDChangedCondition>("LEDChangedCondition");
}