#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__OPENDRAWER_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__OPENDRAWER_BT_NODES_H

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace statemachine
{
    /**
     * @brief A BT::ConditionNode that returns SUCCESS when goal is
     * updated on the blackboard and FAILURE otherwise
     */
    class OpenDrawer : public BT::SyncActionNode
    {
    public:
        OpenDrawer(
            const std::string &name,
            const BT::NodeConfig &config);

        OpenDrawer() = delete;

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
                BT::OutputPort<communication_interfaces::msg::DrawerAddress>(
                    "drawer_address", "address of the drawer thats used to execute the action"),
                BT::InputPort<std::string>(
                    "drawer_open_topic",
                    "/open_drawer",
                    "topic thats used to execute the action")};
        }

    protected:
        std::string topic_name_;
        BT::Blackboard::Ptr blackboard_;
        communication_interfaces::msg::DrawerAddress drawer_address_;
        rclcpp::Publisher<communication_interfaces::msg::DrawerAddress>::SharedPtr open_publisher_;

    private:
        rclcpp::Node::SharedPtr _node;

        // rclcpp::CallbackGroup::SharedPtr _callback_group;
        // rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    };
} // namespace statemachine
#endif