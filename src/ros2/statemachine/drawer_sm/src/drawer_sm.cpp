
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

#include "bt_plugins/behavior_tree_engine.hpp"

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
    "change_led_action_bt_node",
    "drawer_open_request_action_bt_node",
    "open_drawer_action_bt_node",
    "drawer_status_condition_bt_node",
    // "example_plugin"
  };
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration* config_;
  config_ = new BT::NodeConfiguration();

  // Create the blackboard that will be shared by all of the nodes in the tree
  config_->blackboard = BT::Blackboard::create();
  // Put items on the blackboard
  config_->blackboard->set<rclcpp::Node::SharedPtr>(
    "node",
    node_);
  config_->blackboard->set<std::chrono::milliseconds>(
    "bt_loop_duration",
    std::chrono::milliseconds(10));
  auto bt_engine = std::make_unique<BehaviorTreeEngine>(plugins);
  std::cout << "lass mal baum laden"<< std::endl;
  BT::Tree bt = bt_engine->createTreeFromFile("/workspace/src/statemachine/drawer_sm/trees/drawer_sequence_simple.xml", config_->blackboard);
  std::cout << "lüppt" << std::endl;

  rclcpp::shutdown();
  return 0;
}