
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
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  BT::BehaviorTreeFactory factory;
  
  rclcpp::init(argc, argv);
  factory.registerNodeType<DrawerStatusCondition>("DrawerStatusCondition");
  factory.registerNodeType<ChangeLED>("ChangeLED");
  factory.registerNodeType<DrawerOpenReq>("DrawerOpenReq");
  factory.registerNodeType<OpenDrawer>("OpenDrawer");
  factory.registerNodeType<ApproachObject>("ApproachObject");
  std::cout << "print" << std::endl;
  auto tree = factory_->createTreeFromFile("/workspace/src/statemachine/drawer_sm/trees/drawer_sequence_simple.xml");
  std::cout << "was los hier" << std::endl;
  tree.tickRootWhileRunning();
  rclcpp::spin(std::make_shared<drawer_statemachine::BTTicker>(&factory_));
  rclcpp::shutdown();
  return 0;
}