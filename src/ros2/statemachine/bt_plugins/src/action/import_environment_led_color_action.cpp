#include "bt_plugins/action/import_environment_led_color_action.hpp"
#include <cstdlib>
#include <string>
#include <sstream>

namespace bt_plugins
{

  ImportEnvironmentLEDColor::ImportEnvironmentLEDColor(
      const std::string &name,
      const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config)
  {
  }

  BT::NodeStatus ImportEnvironmentLEDColor::tick()
  {
    auto getEnvAsUInt = [](const char *varName, int defaultValue) -> uint8_t
    {
      const char *value = std::getenv(varName);
      if (value)
      {
        int intValue = std::stoi(value);
        if (intValue >= 0 && intValue <= 255)
        {
          return static_cast<uint8_t>(intValue);
        }
      }
      return defaultValue;
    };

    // Get the values from the environment variables
    const uint8_t red = getEnvAsUInt("RED", 0);
    const uint8_t green = getEnvAsUInt("GREEN", 0);
    const uint8_t blue = getEnvAsUInt("BLUE", 0);
    const uint8_t brightness = getEnvAsUInt("BRIGHTNESS", 0);

    setOutput("blue", blue);
    setOutput("red", red);
    setOutput("green", green);
    setOutput("brightness", brightness);

    return BT::NodeStatus::SUCCESS;
  }

} // namespace bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_plugins::ImportEnvironmentLEDColor>("ImportEnvironmentLEDColor");
}
