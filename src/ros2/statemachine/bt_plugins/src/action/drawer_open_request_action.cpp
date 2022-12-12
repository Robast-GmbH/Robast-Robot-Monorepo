#include "bt_plugins/action/drawer_open_request_action.hpp"



namespace drawer_statemachine
{

    using std::placeholders::_1;

    DrawerOpenReq::DrawerOpenReq(
        const std::string& name,
        const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        getInput("drawer_address_topic", topic_name_);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        drawer_open_sub_ = node_->create_subscription<communication_interfaces::msg::DrawerAddress>(
            topic_name_,
            qos,
            std::bind(&DrawerOpenReq::callbackDrawerOpenReq, this, _1),
            sub_option);
        new_message_ = false;
    }

    BT::NodeStatus DrawerOpenReq::tick()
    {
        callback_group_executor_.spin_some();

        if (new_message_)
        {
            RCLCPP_INFO(rclcpp::get_logger("DrawerOpenReq"), "new drawer_address publlished\n drawer_id:%d",drawer_address_.drawer_id);
            setOutput("drawer_address", drawer_address_);
            new_message_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(rclcpp::get_logger("DrawerOpenReq"), "ticked drawer open req");
        return BT::NodeStatus::RUNNING;
    }

    void
        DrawerOpenReq::callbackDrawerOpenReq(const communication_interfaces::msg::DrawerAddress::SharedPtr msg)
    {
        // RCLCPP_DEBUG(rclcpp::get_logger("DrawerOpenReq"), "received request");
        RCLCPP_INFO(rclcpp::get_logger("DrawerOpenReq"), "whatsub");
        drawer_address_.drawer_controller_id = msg->drawer_controller_id;
        drawer_address_.drawer_id = msg->drawer_id;
        new_message_ = true;
    }

}  // namespace drawer_statemachine

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::DrawerOpenReq>("DrawerOpenReq");
}