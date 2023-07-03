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
        DrawerStatusCondition(const std::string &name, const BT::NodeConfig &config);
        DrawerStatusCondition() = delete;

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("target_value", "false"),
                BT::InputPort<std::string>("topic", "/drawer_is_open")};
        }

    protected:
        void callbackDrawerFeedback(const communication_interfaces::msg::DrawerStatus::SharedPtr msg);

    private:
        rclcpp::Node::SharedPtr _node;

        rclcpp::CallbackGroup::SharedPtr _callback_group;
        rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
        rclcpp::Subscription<communication_interfaces::msg::DrawerStatus>::SharedPtr _drawer_status_sub;
        bool _last_message;
        bool _target_value;
        std::string _topic_name;
    };
} // namespace drawer_statemachine

#endif