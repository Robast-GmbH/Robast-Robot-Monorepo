#include "bt_plugins/action/drawer_change_state_request_action.hpp"

namespace statemachine
{

    using std::placeholders::_1;

    DrawerChangeStateReq::DrawerChangeStateReq(
        const std::string &name,
        const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config)
    {
        _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        _callback_group = _node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

        getInput("drawer_address_topic", topic_name_);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = _callback_group;
        _drawer_open_sub = _node->create_subscription<communication_interfaces::msg::DrawerAddress>(
            topic_name_,
            qos,
            std::bind(&DrawerChangeStateReq::callbackDrawerChangeStateReq, this, _1),
            sub_option);
        _new_message = false;
    }

    BT::NodeStatus DrawerChangeStateReq::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus DrawerChangeStateReq::onRunning()
    {
        _callback_group_executor.spin_some();

        if (_new_message)
        {
            RCLCPP_INFO(rclcpp::get_logger("DrawerChangeStateReq"), "new drawer_address publlished\n drawer_id:%d", drawer_address_.drawer_id);
            setOutput("drawer_address", drawer_address_);
            _new_message = false;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    void DrawerChangeStateReq::onHalted()
    {
        RCLCPP_INFO(rclcpp::get_logger("DrawerChangeStateReq"), "action halted");
    }

    void DrawerChangeStateReq::callbackDrawerChangeStateReq(const communication_interfaces::msg::DrawerAddress::SharedPtr msg)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("DrawerChangeStateReq"), "received request");
        drawer_address_.module_id = msg->module_id;
        drawer_address_.drawer_id = msg->drawer_id;
        _new_message = true;
    }

} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<statemachine::DrawerChangeStateReq>("DrawerChangeStateReq");
}