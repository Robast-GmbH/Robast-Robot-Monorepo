#include "bt_plugins/action/set_default_led_color_action.hpp"

namespace bt_plugins
{
  SetDefaultLedColor::SetDefaultLedColor(
      const std::string &name,
      const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
  }

  BT::NodeStatus SetDefaultLedColor::tick()
  {
    // Set the default values
    setOutput("blue", 0);
    setOutput("red", 0);
    setOutput("green", 255);
    setOutput("brightness", 20);
    return BT::NodeStatus::SUCCESS;
  }
} // namespace bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_plugins::SetDefaultLedColor>("SetDefaultLEDColor");
}
