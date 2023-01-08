
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "bt_plugins/action/change_led_action.hpp"
#include "bt_plugins/action/drawer_open_request_action.hpp"
#include "bt_plugins/action/open_drawer_action.hpp"
#include "bt_plugins/condition/drawer_status_condition.hpp"
#include "bt_plugins/action/example_plugin.hpp"

#include "bt_plugins/behavior_tree_engine.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
// #include "nav2_lifecycle_manager/lifecycle_manager.hpp"


// namespace drawer_statemachine
// {
  class BTTicker : public rclcpp::Node
  {
  public:
    BTTicker()
      : Node("bt_ticker")
    {
      timer_ = this->create_wall_timer(
      500ms, std::bind(&BTTicker::ticking, this));

    }

  private:
    void ticking()
    {
      
    }
    rclcpp::TimerBase::SharedPtr timer_;

  };
// }

using namespace drawer_statemachine;

int main(int argc, char* argv[ ])
{
  rclcpp::init(argc, argv);
  const std::vector<std::string> plugins = {
    "Hans",
    "drawer_open_request_action_bt_node",
  };
  // auto node = std::make_shared<nav2_lifecycle_manager::LifecycleManager>();
  

  // "change_led_action_bt_node",
  // "open_drawer_action_bt_node",
  // "drawer_open_request_action_bt_node",
  // "drawer_status_condition_bt_node",
  // "example_action_bt_node"
  rclcpp::Node node = rclcpp::Node("node");
  rclcpp::Node::SharedPtr parent_node = std::make_shared<BTTicker>();
  // auto client_node_ = parent_node.lock();
  static BT::NodeConfig* config_;
  config_ = new BT::NodeConfig();
  // Create the blackboard that will be shared by all of the nodes in the tree
  auto blackboard = BT::Blackboard::create();
  // Put items on the blackboard
  blackboard->set<rclcpp::Node::SharedPtr>(
    "node",
    parent_node);
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
  

  // rclcpp::spin(client_node_);
  BtStatus rc = bt_engine->run(&bt, on_loop, is_canceling, std::chrono::milliseconds(10));
  rclcpp::shutdown();
  return 0;
}