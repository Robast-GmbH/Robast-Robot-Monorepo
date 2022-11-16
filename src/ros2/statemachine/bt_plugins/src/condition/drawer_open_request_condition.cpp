#include <string>
#include <vector>
#include "bt_plugins/condition/drawer_open_request_condition.hpp"
#include "std_msgs/msg/string.hpp"



#include "rclcpp/rclcpp.hpp"

namespace drawer_statemachine
{

    using std::placeholders::_1;

    DrawerOpenReq::DrawerOpenReq(
        const std::string& name,
        const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        getInput("drawer_address_topic", topic_name_);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        drawer_open_sub = node_->create_subscription<communication_interfaces::msg::DrawerAddress>(
            topic_name_,
            qos,
            std::bind(&DrawerOpenReq::callbackDrawerOpenReq, this, _1),
            sub_option);
    }

    BT::NodeStatus DrawerOpenReq::tick()
    {
        callback_group_executor_.spin_some();

        // This behavior always use the last selected planner received from the topic input.
        // When no input is specified it uses the default planner.
        // If the default planner is not specified then we work in "required planner mode":
        // In this mode, the behavior returns failure if the planner selection is not received from
        // the topic input.
        if (last_selected_planner_.empty())
        {
            std::string default_planner;
            getInput("default_planner", default_planner);
            if (default_planner.empty())
            {
                return BT::NodeStatus::FAILURE;
            }
            else
            {
                last_selected_planner_ = default_planner;
            }
        }

        setOutput("selected_planner", last_selected_planner_);

        return BT::NodeStatus::SUCCESS;
    }

    void
        DrawerOpenReq::callbackDrawerOpenReq(const communication_interfaces::msg::DrawerAddress::SharedPtr msg)
    {
        last_selected_planner_ = msg->data;
    }


}  // namespace drawer_statemachine

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::DrawerOpenReq>("DrawerOpenReq");
}