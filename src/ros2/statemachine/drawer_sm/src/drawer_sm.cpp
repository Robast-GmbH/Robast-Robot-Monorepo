
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

namespace drawer_statemachine
{
  class BTTicker : public rclcpp::Node
  {
  public:
    BTTicker(BT::BehaviorTreeFactory* factory)
      : Node("bt_ticker")
    {
      std::cout << "BTTicker" << std::endl;

      std::cout << "hinzugefügt is" << std::endl;

    }

  private:
  };
}

using namespace drawer_statemachine;
int main(int argc, char* argv[ ])
{
  rclcpp::init(argc, argv);
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration* config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_ = std::make_shared<BT::BehaviorTreeFactory>(BT::BehaviorTreeFactory());
  // BT::BehaviorTreeFactory factory;


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

  factory_->registerNodeType<DrawerStatusCondition>("DrawerStatusCondition");
  factory_->registerNodeType<ChangeLED>("ChangeLED");
  factory_->registerNodeType<DrawerOpenReq>("DrawerOpenReq");
  factory_->registerNodeType<OpenDrawer>("OpenDrawer");
  factory_->registerNodeType<ApproachObject>("ApproachObject");
  // std::cout << "print" << std::endl;
  BT::Tree tree = factory_->createTreeFromFile("/workspace/src/statemachine/drawer_sm/trees/drawer_sequence_simple.xml");
  // std::cout << "was los hier" << std::endl;
  tree.tickRootWhileRunning();
  // rclcpp::spin(std::make_shared<drawer_statemachine::BTTicker>(&factory_));
  rclcpp::shutdown();
  return 0;
}