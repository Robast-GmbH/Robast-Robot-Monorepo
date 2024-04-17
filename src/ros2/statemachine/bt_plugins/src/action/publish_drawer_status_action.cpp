#include "bt_plugins/action/publish_drawer_status_action.hpp"

namespace statemachine
{
    // TODO @Tobi: checkout thread safety in the future again. outcommented for now.

    using std::placeholders::_1;

    PublishDrawerStatus::PublishDrawerStatus(
        const std::string &name,
        const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
        _blackboard = config.blackboard;

        getInput("topic_name", _topic_name);
        // std::scoped_lock l(blackboard_->entryMutex());
        _node = _blackboard->get<rclcpp::Node::SharedPtr>("node");
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        // rclcpp::SubscriptionOptions sub_option;
        // sub_option.callback_group = _callback_group;
        _status_publisher = _node->create_publisher<communication_interfaces::msg::DrawerStatus>(_topic_name, qos);
    }
    BT::NodeStatus PublishDrawerStatus::tick()
    {
        communication_interfaces::msg::DrawerAddress drawer_address;
        bool is_open = false;
        getInput("drawer_address", drawer_address);
        getInput("status", is_open);
        RCLCPP_DEBUG(rclcpp::get_logger("PublishDrawerStatus"), "publish value: %d", is_open);
        communication_interfaces::msg::DrawerStatus drawer_status;
        drawer_status.drawer_address = drawer_address;
        drawer_status.drawer_is_open = is_open;
        _status_publisher->publish(drawer_status);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<statemachine::PublishDrawerStatus>("PublishDrawerStatus");
}