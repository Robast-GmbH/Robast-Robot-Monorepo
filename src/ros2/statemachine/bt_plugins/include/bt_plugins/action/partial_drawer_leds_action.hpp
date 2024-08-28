#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__CHANGELED_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__CHANGELED_BT_NODES_H

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/tray_task.hpp"

namespace statemachine
{
    /**
     * @brief A BT::ConditionNode that returns SUCCESS when goal is
     * updated on the blackboard and FAILURE otherwise
     */
    class PartialDrawerLED : public BT::SyncActionNode
    {
    public:
        PartialDrawerLED(
            const std::string &name,
            const BT::NodeConfig &config);

        PartialDrawerLED() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<uint8_t>(
                    "led_brightness_top", 0, "brightness"),
                BT::InputPort<uint8_t>(
                    "led_brightness_mid", 0, "brightness"),
                BT::InputPort<uint8_t>(
                    "led_brightness_bot", 0, "brightness"),
                BT::InputPort<communication_interfaces::msg::DrawerAddress>(
                    "drawer_address", "address of the drawer that should execute the action"),
                BT::InputPort<std::string>(
                    "topic",
                    "/tray_task",
                    "topic thats used to execute the action")};
        }

    protected:
        std::string topic_name_;
        virtual void initializePublisher();

    private:
        BT::Blackboard::Ptr _blackboard;
        rclcpp::Publisher<communication_interfaces::msg::TrayTask>::SharedPtr _tray_led_publisher;
        communication_interfaces::msg::TrayTask _tray_leds;
        rclcpp::Node::SharedPtr _node;
    };
} // namespace statemachine
#endif