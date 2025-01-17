#include "bt_plugins/action/generate_partial_position_action.hpp"

namespace statemachine
{
  GeneratePartialPosition::GeneratePartialPosition(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    getInput("tray_count", _tray_count);
    getInput("front_offset", _front_offset);
  }

  BT::NodeStatus GeneratePartialPosition::tick()
  {
    communication_interfaces::msg::DrawerAddress drawer_address;
    getInput("drawer_address", drawer_address);
    const uint8_t tray_id = drawer_address.drawer_id;
    if (tray_id == 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GeneratePartialPosition"), "Tray ID cannot be 0. First tray is 1.");
      return BT::NodeStatus::FAILURE;
    }

    if (LAST_LID_POSITION <= _front_offset)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GeneratePartialPosition"),
                   "Front offset cannot be greater than last position, which is %d.",
                   LAST_LID_POSITION);
      return BT::NodeStatus::FAILURE;
    }

    if (_tray_count == 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GeneratePartialPosition"), "Tray count cannot be 0.");
      return BT::NodeStatus::FAILURE;
    }

    // Because the first tray is the one in the last position, we need to reverse the order
    const uint8_t reversed_tray_id = (_tray_count + 1) - tray_id;

    const uint8_t target_position =
        _front_offset + (reversed_tray_id * (LAST_LID_POSITION - _front_offset) / _tray_count);

    setOutput("target_position", target_position);
    return BT::NodeStatus::SUCCESS;
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::GeneratePartialPosition>("GeneratePartialPosition");
}
