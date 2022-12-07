
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "bt_plugins/action/change_led_action.hpp"
#include "bt_plugins/action/drawer_open_request_action.hpp"
#include "bt_plugins/action/open_drawer_action.hpp"
#include "bt_plugins/condition/drawer_status_condition.hpp"
#include "bt_plugins/example_plugin.hpp"

#include "../include/drawer_sm/behavior_tree_engine.hpp"

// namespace drawer_statemachine
// {
//   class BTTicker : public rclcpp::Node
//   {
//   public:
//     BTTicker(BT::BehaviorTreeFactory* factory)
//       : Node("bt_ticker")
//     {
//       std::cout << "BTTicker" << std::endl;

//       std::cout << "hinzugefügt is" << std::endl;

//     }

//   private:
//   };
// }

using namespace drawer_statemachine;
int main(int argc, char* argv[ ])
{
  rclcpp::init(argc, argv);
  const std::vector<std::string> plugins = {
    "change_led_action",
    "drawer_open_request_action",
    "open_drawer_action",
    "drawer_status_condition",
    "example_plugin"
  };
  auto bt_engine = std::make_unique<BehaviorTreeEngine>(plugins);
  rclcpp::shutdown();
  return 0;
}