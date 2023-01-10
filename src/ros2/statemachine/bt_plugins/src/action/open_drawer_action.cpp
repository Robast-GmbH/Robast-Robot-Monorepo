#include "bt_plugins/action/open_drawer_action.hpp"



namespace drawer_statemachine
{

    using std::placeholders::_1;

    OpenDrawer::OpenDrawer(
        const std::string& name,
        const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        // callback_group_ = node_->create_callback_group(
        //     rclcpp::CallbackGroupType::MutuallyExclusive,
        //     false);
        // callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        getInput("drawer_open_topic", topic_name_);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        rclcpp::SubscriptionOptions sub_option;
        // sub_option.callback_group = callback_group_;
        open_publisher_ = node_->create_publisher<communication_interfaces::msg::DrawerAddress>(topic_name_, qos);
    }

    BT::NodeStatus OpenDrawer::onStart(){
        return BT::NodeStatus::RUNNING;
    }
    
    BT::NodeStatus OpenDrawer::onRunning()
    {
        communication_interfaces::msg::DrawerAddress drawer_address;
        getInput("drawer_address", drawer_address);
        RCLCPP_DEBUG(rclcpp::get_logger("OpenDrawer"), "lass mal drawer aufmachen");

        getInput("drawer_address", drawer_address);
        open_publisher_->publish(drawer_address);
        
        return BT::NodeStatus::SUCCESS;
    }

    void OpenDrawer::onHalted()
    {
        open_publisher_.reset();
        RCLCPP_DEBUG(rclcpp::get_logger("OpenDrawer"), "publisher resetted");
    }

}  // namespace drawer_statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::OpenDrawer>("OpenDrawer");
}