#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__DRAWEROPENREQ_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__DRAWEROPENREQ_BT_NODES_H

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
    class DrawerOpenReq: public BT::StatefulActionNode
    {
    public:
        DrawerOpenReq(
            const std::string& name,
            const BT::NodeConfig& config);

        DrawerOpenReq() = delete;

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
              BT::InputPort<std::string>("drawer_address_topic", "topic", "empty"),
              BT::OutputPort<communication_interfaces::msg::DrawerAddress>("drawer_address", "topic")
            };
        }


    protected:
        std::string topic_name_;
        void callbackDrawerOpenReq(const communication_interfaces::msg::DrawerAddress::SharedPtr msg);

    private:
        rclcpp::Node::SharedPtr _node;

        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<communication_interfaces::msg::DrawerAddress>::SharedPtr drawer_open_sub_;
        communication_interfaces::msg::DrawerAddress drawer_address_;
        bool new_message_;
    };
}
#endif 