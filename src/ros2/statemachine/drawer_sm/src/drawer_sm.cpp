
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

#include "bt_plugins/behavior_tree_engine.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"

#include "std_msgs/msg/empty.hpp"
// #include "nav2_lifecycle_manager/lifecycle_manager.hpp"


// namespace drawer_statemachine
// {
  class BTTicker : public rclcpp::Node
  {
  public:
    BTTicker():Node("bt_tickers")
    {
      const std::vector<std::string> plugins = {
        "Hans",
        "drawer_open_request_action_bt_node",
      };
      static BT::NodeConfig* config_;
      config_ = new BT::NodeConfig();
      // Create the blackboard that will be shared by all of the nodes in the tree
      auto blackboard = BT::Blackboard::create();
      // Put items on the blackboard
      blackboard->set<rclcpp::Node::SharedPtr>(
        "node",
        std::shared_ptr<rclcpp::Node>(this));
      blackboard->set<std::chrono::milliseconds>(
        "bt_loop_duration",
        std::chrono::milliseconds(10));
      auto bt_engine = std::make_unique<drawer_statemachine::BehaviorTreeEngine>(plugins);
      bt_ = bt_engine->createTreeFromFile("/workspace/install/drawer_sm/trees/trees/drawer_sequence_simple.xml", blackboard);
      // using namespace std::chrono_literals;
      // timer_ = this->create_wall_timer(
      // 500ms, std::bind(&BTTicker::ticking, this));
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local().reliable();

      start_bt_sub_ = this->create_subscription<std_msgs::msg::Empty>(
          "start_bt",
          qos,
          std::bind(&BTTicker::callbackRunBT, this, std::placeholders::_1)   );

      
    }

  private:
    void ticking()
    {
      std::cout << "tick" << std::endl;
      bt_.tickOnce();
      std::cout << "ticked" << std::endl;
    }
    void
        callbackRunBT(const std_msgs::msg::Empty msg)
    {
      using namespace std::chrono_literals;
      bt_.tickWhileRunning(500ms);
      return;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    // drawer_statemachine::BehaviorTreeEngine bt_engine_;
    BT::Tree bt_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_bt_sub_;

  };
// }

using namespace drawer_statemachine;

int main(int argc, char* argv[ ])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTTicker>());
  rclcpp::shutdown();
  return 0;
}