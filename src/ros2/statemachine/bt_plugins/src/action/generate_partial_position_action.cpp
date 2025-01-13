#include "bt_plugins/action/generate_partial_position_action.hpp"

namespace statemachine
{
  GeneratePartialPosition::GeneratePartialPosition(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
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

    uint8_t first_tray_position;
    getInput("first_tray_position", first_tray_position);

    uint8_t last_tray_position;
    getInput("last_tray_position", last_tray_position);
    if (last_tray_position <= first_tray_position)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GeneratePartialPosition"),
                   "Last tray position must be greater than first tray position.");
      return BT::NodeStatus::FAILURE;
    }

    uint8_t tray_count;
    getInput("tray_count", tray_count);
    if (tray_count == 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GeneratePartialPosition"), "Tray count cannot be 0.");
      return BT::NodeStatus::FAILURE;
    }

    uint8_t target_position =
        first_tray_position + (tray_id - 1) * (last_tray_position - first_tray_position) / (tray_count - 1);
    setOutput("target_position", target_position);
    return BT::NodeStatus::SUCCESS;
  }
}   // namespace statemachine

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::GeneratePartialPosition>("GeneratePartialPosition");
}
