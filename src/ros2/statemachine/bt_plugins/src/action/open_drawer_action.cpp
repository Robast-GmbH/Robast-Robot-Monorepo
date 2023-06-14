#include "bt_plugins/action/open_drawer_action.hpp"

namespace drawer_statemachine
{

    using std::placeholders::_1;

    OpenDrawer::OpenDrawer(
        const std::string &name,
        const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config)
    {
        _blackboard = config.blackboard;

        getInput("drawer_open_topic", topic_name_);
        // std::scoped_lock l(_blackboard->entryMutex());
        _node = _blackboard->get<rclcpp::Node::SharedPtr>("node");
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        rclcpp::SubscriptionOptions sub_option;
        // sub_option.callback_group = _callback_group;
        open_publisher_ = _node->create_publisher<communication_interfaces::msg::DrawerAddress>(topic_name_, qos);
    }

    BT::NodeStatus OpenDrawer::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus OpenDrawer::onRunning()
    {
        // std::scoped_lock l(_blackboard->entryMutex());
        drawer_address_ = _blackboard->get<communication_interfaces::msg::DrawerAddress>("drawer_address");
        setOutput("drawer_address", drawer_address_);
        RCLCPP_DEBUG(rclcpp::get_logger("OpenDrawer"), "open drawer ");
        open_publisher_->publish(drawer_address_);

        return BT::NodeStatus::SUCCESS;
    }

    void OpenDrawer::onHalted()
    {
        open_publisher_.reset();
        RCLCPP_DEBUG(rclcpp::get_logger("OpenDrawer"), "publisher resetted");
    }

} // namespace drawer_statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::OpenDrawer>("OpenDrawer");
}