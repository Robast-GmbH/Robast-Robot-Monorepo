#include "bt_plugins/action/open_drawer_action.hpp"



namespace drawer_statemachine
{

    using std::placeholders::_1;

    OpenElectricDrawer::OpenElectricDrawer(
        const std::string& name,
        const BT::NodeConfig& config)
        : OpenDrawer(name, config)
    {
        _blackboard = config.blackboard;


        // std::scoped_lock l(_blackboard->entryMutex());
        _node = _blackboard->get<rclcpp::Node::SharedPtr>("node");
        // drawer_address_ = _blackboard->get<communication_interfaces::msg::DrawerAddress>("drawer_address");
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();

        getInput("move_electric_drawer_topic", topic_name_);
        getInput("goto_position", drawer_task_.drawer_address);
        getInput("speed_mode", drawer_task_.blue);
        getInput("stall_guard_enable", drawer_task_.red);        
    }

    void ChangeLED::initializePublisher()
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local().reliable();
        _move_electric_drawer_publisher = _node->create_publisher<communication_interfaces::msg::DrawerTask>(topic_name_, qos);
    }
    
    BT::NodeStatus OpenElectricDrawer::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus OpenElectricDrawer::onRunning()
    {
        // std::scoped_lock l(_blackboard->entryMutex());
        drawer_task_.drawer_address = _blackboard->get<communication_interfaces::msg::DrawerAddress>("drawer_address");
        setOutput("drawer_address", drawer_task_drawer_address);
        RCLCPP_DEBUG(rclcpp::get_logger("OpenElectricDrawer"), "open drawer ");
        _move_electric_drawer_publisher->publish(drawer_address_);

        return BT::NodeStatus::SUCCESS;
    }

    void OpenElectricDrawer::onHalted()
    {
        open_publisher_.reset();
        RCLCPP_DEBUG(rclcpp::get_logger("OpenElectricDrawer"), "publisher resetted");
    }

}  // namespace drawer_statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::OpenDrawer>("OpenDrawer");
}