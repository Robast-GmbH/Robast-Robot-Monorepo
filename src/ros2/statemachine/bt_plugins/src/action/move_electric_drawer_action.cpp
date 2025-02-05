#include "bt_plugins/action/move_electric_drawer_action.hpp"

namespace statemachine
{

  using std::placeholders::_1;

  MoveElectricDrawer::MoveElectricDrawer(const std::string &name, const BT::NodeConfig &config)
      : BT::StatefulActionNode(name, config)
  {
    blackboard_ = config.blackboard;

    _node = blackboard_->get<rclcpp::Node::SharedPtr>("node");

    getInput("move_electric_drawer_topic", topic_name_);
    getInput("speed", drawer_task_.speed);
    getInput("stall_guard_value", drawer_task_.stall_guard_value);

    initializePublisher();
  }

  void MoveElectricDrawer::initializePublisher()
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();
    _move_electric_drawer_publisher =
        _node->create_publisher<communication_interfaces::msg::DrawerTask>(topic_name_, qos);
  }

  BT::NodeStatus MoveElectricDrawer::onStart()
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus MoveElectricDrawer::onRunning()
  {
    getInput("target_position", drawer_task_.target_position);
    getInput("drawer_address", drawer_task_.drawer_address);
    RCLCPP_DEBUG(rclcpp::get_logger("MoveElectricDrawer"), "move drawer ");
    _move_electric_drawer_publisher->publish(drawer_task_);
    return BT::NodeStatus::SUCCESS;
  }

  void MoveElectricDrawer::onHalted()
  {
    _move_electric_drawer_publisher.reset();
    RCLCPP_DEBUG(rclcpp::get_logger("MoveElectricDrawer"), "publisher resetted");
  }

}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::MoveElectricDrawer>("MoveElectricDrawer");
}