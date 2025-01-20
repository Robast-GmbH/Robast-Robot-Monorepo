#ifndef STATEMACHINE__BT_PLUGINS__ACTION__IMPORT_ENVIRONMENT_LED_COLOR_BT_NODES_H
#define STATEMACHINE__BT_PLUGINS__ACTION__IMPORT_ENVIRONMENT_LED_COLOR_BT_NODES_H

#include <string>
#include <cstdint>

#include "behaviortree_cpp/action_node.h"

namespace bt_plugins
{
    class ImportEnvironmentLEDColor : public BT::SyncActionNode
    {
    public:
        ImportEnvironmentLEDColor(
            const std::string &name,
            const BT::NodeConfig &config);

        ImportEnvironmentLEDColor() = delete;

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<uint8_t>(
                    "blue"),
                BT::OutputPort<uint8_t>(
                    "red"),
                BT::OutputPort<uint8_t>(
                    "green"),
                BT::OutputPort<uint8_t>(
                    "brightness")};
        }
    };
} // namespace bt_plugins
#endif