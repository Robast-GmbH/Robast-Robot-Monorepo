#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__PUBLISHDRAWERSTATUS_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__PUBLISHDRAWERSTATUS_BT_NODES_H

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "std_msgs/msg/bool.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"

namespace statemachine
{
    /**
     * @brief A BT::ConditionNode that returns SUCCESS when goal is
     * updated on the blackboard and FAILURE otherwise
     */
    class PublishDrawerStatus : public BT::SyncActionNode
    {
    public:
        PublishDrawerStatus(
            const std::string &name,
            const BT::NodeConfig &config);

        PublishDrawerStatus() = delete;

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
                BT::InputPort<bool>(
                    "status", true, "is_open status of the drawer thats used to execute the action"),
                BT::InputPort<communication_interfaces::msg::DrawerAddress>(
                    "drawer_address",
                    "address of the drawer thats used to execute the action"),
                BT::InputPort<std::string>(
                    "topic_name",
                    "/bt_drawer_open",
                    "topic thats used to execute the action")};
        }

    private:
        std::string _topic_name;
        BT::Blackboard::Ptr _blackboard;
        rclcpp::Node::SharedPtr _node;
        rclcpp::Publisher<communication_interfaces::msg::DrawerStatus>::SharedPtr _status_publisher;

        // rclcpp::CallbackGroup::SharedPtr _callback_group;
        // rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    };
} // namespace statemachine
#endif