
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


class BTTicker: public rclcpp::Node
  {
  public:
    BTTicker():Node("bt_tickers")
    {
      const std::vector<std::string> plugins = {
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
      std::string path = "/workspace/install/drawer_sm/trees/trees/drawer_sequence_simple.xml";
      bt_ = bt_engine->createTreeFromFile(path, blackboard);
      if (bt_.subtrees[0])
      {
        for (auto plugin_name = plugins.begin(); plugin_name != plugins.end(); plugin_name++)
        {
          bool found = false;
          auto iter = bt_.subtrees[0]->nodes.begin();
          for (; iter != bt_.subtrees[0]->nodes.end(); iter++)
          {
            if ((*iter)->registrationName() == "DrawerOpenReq")
            {
              drawer_statemachine::DrawerOpenReq* node = dynamic_cast<drawer_statemachine::DrawerOpenReq*> ((*iter).get());
              std::cout << "found: " << *plugin_name << "\n";
              found = true;
              
              break;
            }
          }
          if (found)
          {
            std::cout << "fuck yea" << "\n";
          }
        }          
      }
      
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



int main(int argc, char* argv[ ])
{
  using namespace drawer_statemachine;
  rclcpp::init(argc, argv);
  const std::vector<std::string> plugins = {
                "change_led_action_bt_node",
  };
  static rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test");
  static BT::NodeConfig* config_;
  config_ = new BT::NodeConfig();
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
  communication_interfaces::msg::DrawerLeds expected_drawer_led;
  expected_drawer_led.red = 5;
  expected_drawer_led.green = 255;
  expected_drawer_led.blue = 10;
  expected_drawer_led.mode = 1;
  expected_drawer_led.brightness = 128;
  std::string led_tree_xml =
          R"(
          <root BTCPP_format="4" >
              <BehaviorTree ID="MainTree">
                  <ChangeLED 
                      blue="10"
                      brightness="128"
                      drawer_address="{drawer}"
                      green="255"
                      led_topic="drawer_leds"
                      mode="1"
                      red="5"/>
              </BehaviorTree>
          </root>)";
  

  blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node);
  auto bt_engine = std::make_unique<drawer_statemachine::BehaviorTreeEngine>(plugins);
  auto bt = bt_engine->createTreeFromText(led_tree_xml, blackboard);
  
  rclcpp::spin(std::make_shared<BTTicker>());
  rclcpp::shutdown();
  return 0;
}