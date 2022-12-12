
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
#include "bt_plugins/action/example_plugin.hpp"

#include "bt_plugins/behavior_tree_engine.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

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
    "Hans",
    "drawer_open_request_action_bt_node",
  };
  // "change_led_action_bt_node",
  // "open_drawer_action_bt_node",
  // "drawer_open_request_action_bt_node",
  // "drawer_status_condition_bt_node",
  // "example_action_bt_node"
  
  auto client_node_ = std::make_shared<rclcpp::Node>("tree_node");
  static BT::NodeConfiguration* config_;
  config_ = new BT::NodeConfiguration();

  // Create the blackboard that will be shared by all of the nodes in the tree
  auto blackboard = BT::Blackboard::create();
  // Put items on the blackboard
  blackboard->set<rclcpp::Node::SharedPtr>(
    "node",
    client_node_);
  blackboard->set<std::chrono::milliseconds>(
    "bt_loop_duration",
    std::chrono::milliseconds(10));
  auto bt_engine = std::make_unique<BehaviorTreeEngine>(plugins);

  BT::Tree bt = bt_engine->createTreeFromFile("/workspace/install/drawer_sm/trees/trees/drawer_sequence_simple.xml", blackboard);

  auto is_canceling = [&]() {
    return false;
  };

  auto on_loop = [&]() {
    // std::cout << "on_loop" <<std::endl;
  };

  // auto bt_action_server_ = std::make_unique<BtActionServer<std::string>>(
  //   client_node_,
  //   "getName()",
  //   plugins,
  //   "/workspace/install/drawer_sm/trees/trees/drawer_sequence_simple.xml",
  //   std::bind([&]() {return true;}),
  //   std::bind([&]() {return true;}),
  //   std::bind([&]() {return true;}),
  //   std::bind([&]() {return true;}));
  

  BtStatus rc = bt_engine->run(&bt, on_loop, is_canceling, std::chrono::milliseconds(10));
  rclcpp::shutdown();
  return 0;
}