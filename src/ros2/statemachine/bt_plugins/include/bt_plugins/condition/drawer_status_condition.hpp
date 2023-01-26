#ifndef DRAWERSTATUS_PLUGINS_CONDITION_HPP_
#define DRAWERSTATUS_PLUGINS_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "communication_interfaces/msg/drawer_status.hpp"

namespace drawer_statemachine
{
    class DrawerStatusCondition : public BT::ConditionNode
    {
        public:
            DrawerStatusCondition(const std::string& name, const BT::NodeConfig& config);
            DrawerStatusCondition() = delete;

            BT::NodeStatus tick() override;

            static BT::PortsList providedPorts()
            {
                return
                {
                    BT::InputPort<bool>("target_value", "false"),
                    BT::InputPort<std::string>("topic", "/drawer_is_open")
                };
            }
    protected:
        void callbackDrawerFeedback(const communication_interfaces::msg::DrawerStatus::SharedPtr msg);

    private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr callback_group_;
            rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
            rclcpp::Subscription<communication_interfaces::msg::DrawerStatus>::SharedPtr drawer_status_sub_;
            bool last_message_;
            bool target_value_;
            std::string topic_name_;
            // communication_interfaces::msg::DrawerAddress drawer_address_;

    };
}

#endif