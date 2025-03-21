#ifndef STATEMACHINE__BT_PLUGINS__ACTION__PARTIAL_COLORIZE_LEDS_BT_NODES_H
#define STATEMACHINE__BT_PLUGINS__ACTION__PARTIAL_COLORIZE_LEDS_BT_NODES_H

#include <string>
#include <vector>
#include <memory>
#include <cstdint> // For uint8_t
#include "bt_base_types/led_base.hpp"

#include "behaviortree_cpp/action_node.h"

namespace bt_plugins
{
    class PartialColorizeLED : public BT::SyncActionNode
    {
    public:
        PartialColorizeLED(
            const std::string &name,
            const BT::NodeConfig &config);

        PartialColorizeLED() = delete;

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<uint8_t>(
                    "blue", 0, "blue"),
                BT::InputPort<uint8_t>(
                    "red", 0, "red"),
                BT::InputPort<uint8_t>(
                    "green", 0, "green"),
                BT::InputPort<uint8_t>(
                    "brightness", 0, "brightness"),
                BT::InputPort<double>(
                    "upper_bound", 0.0, "percentage"),
                BT::InputPort<double>(
                    "lower_bound", 0.0, "percentage"),
                BT::InputPort<std::vector<bt_base_types::LED>>("LEDs"),
                BT::OutputPort<std::vector<bt_base_types::LED>>("LEDs_colored")};
        }
    };
} // namespace bt_plugins
#endif