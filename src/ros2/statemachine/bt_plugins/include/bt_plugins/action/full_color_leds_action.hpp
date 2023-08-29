#ifndef statemachine__BT_PLUGINS__ACTION__BASELED_BT_NODES_H
#define statemachine__BT_PLUGINS__ACTION__BASELED_BT_NODES_H

#include <string>
#include <vector>
#include <memory>
#include <cstdint> // For uint8_t
#include "bt_base_types/LED_base.hpp"

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"

namespace base_plugins
{
    class ChangeStaticLEDColor : public BT::SyncActionNode
    {
    public:
        ChangeStaticLEDColor(
            const std::string &name,
            const BT::NodeConfig &config);

        ChangeStaticLEDColor() = delete;

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
                BT::InputPort<bt_base_types::LED>("LED_color"),
                BT::OutputPort<bt_base_types::LED>("LED_color")};
        }

    private:
    };
} // namespace statemachine
#endif