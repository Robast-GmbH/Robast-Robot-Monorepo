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
    auto getEnvAsInt = [](const char *varName, int defaultValue) -> uint8_t
    {
      const char *value = std::getenv(varName);
      if (value)
      {
        std::istringstream iss(value);
        uint8_t intValue;
        if (iss >> intValue)
        {
          return intValue;
        }
      }
      return defaultValue;
    };

    // Get the values from the environment variables
    uint8_t red = getEnvAsInt("RED", 0);
    uint8_t green = getEnvAsInt("GREEN", 0);
    uint8_t blue = getEnvAsInt("BLUE", 0);
    uint8_t brightness = getEnvAsInt("BRIGHTNESS", 0);

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