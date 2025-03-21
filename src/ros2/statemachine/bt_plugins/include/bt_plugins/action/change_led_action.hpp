#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__CHANGELED_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__CHANGELED_BT_NODES_H

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"

namespace statemachine
{
    /**
     * @brief A BT::ConditionNode that returns SUCCESS when goal is
     * updated on the blackboard and FAILURE otherwise
     */
    class ChangeLED : public BT::SyncActionNode
    {
    public:
        ChangeLED(
            const std::string &name,
            const BT::NodeConfig &config);

        ChangeLED() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        const communication_interfaces::msg::DrawerLeds getDrawerLED();

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
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
                BT::InputPort<uint8_t>(
                    "mode", 0, "mode of the led animation"),
                BT::InputPort<bool>(
                    "use_blackboard_address", false, "wether to use the address that's passed around by I/O or blackboard"),
                BT::InputPort<communication_interfaces::msg::DrawerAddress>(
                    "drawer_address", "address of the drawer that should execute the action"),
                BT::InputPort<std::string>(
                    "led_topic",
                    "/drawer_leds",
                    "topic thats used to execute the action")};
        }

    protected:
        std::string topic_name_;
        virtual void publish();
        virtual void initializePublisher();
        BT::Blackboard::Ptr blackboard_;
        rclcpp::Publisher<communication_interfaces::msg::DrawerLeds>::SharedPtr led_publisher_;
        communication_interfaces::msg::DrawerLeds drawer_leds_;

    private:
        rclcpp::Node::SharedPtr _node;
    };
} // namespace statemachine
#endif