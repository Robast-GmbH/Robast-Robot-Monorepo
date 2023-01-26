#ifndef OPENDRAWER_BT_NODES_H
#define OPENDRAWER_BT_NODES_H

#include <string>
#include <vector>
#include <memory>


#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace drawer_statemachine
{
    /**
     * @brief A BT::ConditionNode that returns SUCCESS when goal is
     * updated on the blackboard and FAILURE otherwise
     */
    class OpenDrawer : public BT::StatefulActionNode
    {
    public:
        OpenDrawer(
            const std::string& name,
            const BT::NodeConfig& config);

        OpenDrawer() = delete;

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
            return
            {
                BT::OutputPort<communication_interfaces::msg::DrawerAddress>(
                    "drawer_address","address of the drawer thats used to execute the action"
                ),
                BT::InputPort<std::string>(
                    "drawer_open_topic",
                    "/open_drawer",
                    "topic thats used to execute the action"
                )
            };
        }


    protected:
        std::string topic_name_;
        BT::Blackboard::Ptr _blackboard;

    private:
        rclcpp::Node::SharedPtr node_;
        communication_interfaces::msg::DrawerAddress drawer_address_;

        // rclcpp::CallbackGroup::SharedPtr callback_group_;
        // rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Publisher<communication_interfaces::msg::DrawerAddress>::SharedPtr open_publisher_;
    };
}
#endif 