#include "bt_plugins/action/get_blackboard_value_action.hpp"

namespace bt_plugins
{
  GetBlackboardEntry::GetBlackboardEntry(
      const std::string &name,
      const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
    _blackboard = config.blackboard;
    getInput("key", _key);
  }

  BT::NodeStatus GetBlackboardEntry::tick()
  {
    communication_interfaces::msg::DrawerAddress hardware_id = _blackboard->get<communication_interfaces::msg::DrawerAddress>(_key);
    std::string value_string = "ticked with address: " + std::to_string(hardware_id.drawer_id) + " " + std::to_string(hardware_id.module_id);
    RCLCPP_DEBUG(rclcpp::get_logger("GetBlackboardEntry"), value_string.c_str());
    setOutput("value", hardware_id);
    return BT::NodeStatus::SUCCESS;
  }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_plugins::GetBlackboardEntry>("GetBlackboardEntry");
}