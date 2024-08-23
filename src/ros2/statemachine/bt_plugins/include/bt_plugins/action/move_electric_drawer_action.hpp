#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__MOVE_ELECTRIC_DRAWER_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__MOVE_ELECTRIC_DRAWER_BT_NODES_H

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_task.hpp"

namespace statemachine
{
    /**
     * @brief A BT::ConditionNode that returns SUCCESS when goal is
     * updated on the blackboard and FAILURE otherwise
     */
    class MoveElectricDrawer : public BT::StatefulActionNode
    {
    public:
        MoveElectricDrawer(
            const std::string &name,
            const BT::NodeConfig &config);

        MoveElectricDrawer() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<communication_interfaces::msg::DrawerAddress>(
                    "drawer_address", "address of the drawer thats used to execute the action"),

                BT::InputPort<uint8_t>(
                    "target_position", 0, "goal position between 0-255"),
                BT::InputPort<uint8_t>(
                    "speed", 0, "speed between 0-255"),
                BT::InputPort<uint8_t>(
                    "stall_guard_value", 0, "value between 0-255, 0 disables stall guard, higher value increases stall sensitivity"),

                BT::InputPort<std::string>(
                    "move_electric_drawer_topic",
                    "/move_drawer",
                    "topic thats used to execute the action")};
        }

    protected:
        std::string topic_name_;
        BT::Blackboard::Ptr blackboard_;
        communication_interfaces::msg::DrawerTask drawer_task_;
        void initializePublisher();

    private:
        rclcpp::Node::SharedPtr _node;
        rclcpp::Publisher<communication_interfaces::msg::DrawerTask>::SharedPtr _move_electric_drawer_publisher;
    };
}
#endif