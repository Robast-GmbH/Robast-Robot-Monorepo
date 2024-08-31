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
    auto getEnvAsInt = [](const char *varName, int defaultValue) -> int
    {
      const char *value = std::getenv(varName);
      if (value)
      {
        std::istringstream iss(value);
        int intValue;
        if (iss >> intValue)
        {
          return intValue;
        }
      }
      return defaultValue;
    };

    // Get the values from the environment variables
    int red = getEnvAsInt("RED", 0);
    int green = getEnvAsInt("GREEN", 0);
    int blue = getEnvAsInt("BLUE", 0);
    int brightness = getEnvAsInt("BRIGHTNESS", 0);

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