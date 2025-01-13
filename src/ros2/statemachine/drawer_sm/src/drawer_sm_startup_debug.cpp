
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// #include "bt_plugins/action/change_led_action.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "bt_plugins/action/drawer_change_state_request_action.hpp"
#include "bt_plugins/action/nfc_to_drawer_action.hpp"
#include "bt_plugins/action/open_drawer_action.hpp"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "bt_plugins/condition/drawer_status_condition.hpp"
#include "error_utils/generic_error_converter.hpp"
#include "std_msgs/msg/empty.hpp"

class BTTest : public rclcpp::Node
{
 public:
  BTTest() : Node("bt_test")
  {
  }
  void configure()
  {
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    const std::vector<std::string> plugins = {
        "nfc_to_drawer_action_bt_node",
    };
    communication_interfaces::msg::DrawerAddress drawer_address;
    drawer_address.drawer_id = 1;
    drawer_address.module_id = 2;
    std::string stringer = error_utils::message_to_string<communication_interfaces::msg::DrawerAddress>(drawer_address);
    std::cout << stringer << std::endl;
    communication_interfaces::msg::DrawerAddress drawer_address2;
    drawer_address2 = error_utils::string_to_message<communication_interfaces::msg::DrawerAddress>(stringer);
    static BT::NodeConfig *config_;
    config_ = new BT::NodeConfig();
    auto blackboard = BT::Blackboard::create();
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    std::string nfc_tree_xml =
        R"(
                <root BTCPP_format="4" >
                    <BehaviorTree ID="MainTree">
                        <NFCToDrawer/>
                    </BehaviorTree>
                </root>)";

    auto tmp = shared_from_this();
    blackboard->set<rclcpp::Node::SharedPtr>("node", std::shared_ptr<rclcpp::Node>(tmp));

    std::map<std::string, DrawerAddress> nfc_dictionary{{}};
    for (int i = 1; i <= 5; i++)
    {
      DrawerAddress drawer_address;
      drawer_address.module_id = 0;
      drawer_address.drawer_id = i;
      nfc_dictionary[std::to_string(i)] = drawer_address;
    }

    blackboard->set<std::map<std::string, DrawerAddress>>("nfc_keys", nfc_dictionary);

    auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
    bt_ = bt_engine->createTreeFromText(nfc_tree_xml, blackboard, "MainTree");
    if (bt_.subtrees[0])
    {
      for (auto plugin_name = plugins.begin(); plugin_name != plugins.end(); plugin_name++)
      {
        bool found = false;
        auto iter = bt_.subtrees[0]->nodes.begin();
        for (; iter != bt_.subtrees[0]->nodes.end(); iter++)
        {
          if ((*iter)->registrationName() == "NFCToDrawer")
          {
            statemachine::NFCToDrawer *node = dynamic_cast<statemachine::NFCToDrawer *>((*iter).get());
            std::cout << "found: " << *plugin_name << "\n";
            found = true;

            break;
          }
        }
        if (found)
        {
          blackboard->set<std::string>("user_access_name", "1");
          bt_.tickOnce();
          auto blackboard_drawer_address = blackboard->get<DrawerAddress>("drawer_address");
          if (blackboard_drawer_address == nfc_dictionary["1"])
          {
            std::cout << "fuck yea"
                      << "\n";
          }
        }
      }
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    _start_bt_sub = this->create_subscription<std_msgs::msg::Empty>(
        "start_bt", qos, std::bind(&BTTest::callbackRunBT, this, std::placeholders::_1));
  }

 private:
  void ticking()
  {
    std::cout << "tick" << std::endl;
    bt_.tickOnce();
    std::cout << "ticked" << std::endl;
  }
  void callbackRunBT(const std_msgs::msg::Empty msg)
  {
    using namespace std::chrono_literals;
    bt_.tickWhileRunning(500ms);
    return;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  // statemachine::BehaviorTreeEngine bt_engine_;
  BT::Tree bt_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _start_bt_sub;
};

class BTTicker : public rclcpp::Node
{
 public:
  BTTicker() : Node("bt_tickers")
  {
  }
  void configure()
  {
    const std::vector<std::string> plugins = {
        "drawer_change_state_request_action_bt_node",
    };
    static BT::NodeConfig *config_;
    config_ = new BT::NodeConfig();
    // Create the blackboard that will be shared by all of the nodes in the tree
    auto blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    auto tmp = shared_from_this();
    blackboard->set<rclcpp::Node::SharedPtr>("node", std::shared_ptr<rclcpp::Node>(tmp));
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);

    std::string default_path = "/workspace/install/drawer_sm/trees/trees/drawer_sequence_simple.xml";
    bt_ = bt_engine->createTreeFromFile(default_path, blackboard, "MainTree");
    if (bt_.subtrees[0])
    {
      for (auto plugin_name = plugins.begin(); plugin_name != plugins.end(); plugin_name++)
      {
        bool found = false;
        auto iter = bt_.subtrees[0]->nodes.begin();
        for (; iter != bt_.subtrees[0]->nodes.end(); iter++)
        {
          if ((*iter)->registrationName() == "DrawerChangeStateReq")
          {
            statemachine::DrawerChangeStateReq *node =
                dynamic_cast<statemachine::DrawerChangeStateReq *>((*iter).get());
            std::cout << "found: " << *plugin_name << "\n";
            found = true;

            break;
          }
        }
        if (found)
        {
          std::cout << "fuck yea"
                    << "\n";
        }
      }
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    _start_bt_sub = this->create_subscription<std_msgs::msg::Empty>(
        "start_bt", qos, std::bind(&BTTicker::callbackRunBT, this, std::placeholders::_1));
  }

 private:
  void ticking()
  {
    std::cout << "tick" << std::endl;
    bt_.tickOnce();
    std::cout << "ticked" << std::endl;
  }
  void callbackRunBT(const std_msgs::msg::Empty msg)
  {
    using namespace std::chrono_literals;
    bt_.tickWhileRunning(500ms);
    return;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  // statemachine::BehaviorTreeEngine bt_engine_;
  BT::Tree bt_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _start_bt_sub;
};

int main(int argc, char *argv[])
{
  using namespace statemachine;
  rclcpp::init(argc, argv);
  // const std::vector<std::string> plugins = {
  //               "change_led_action_bt_node",
  // };
  // static rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test");
  // static BT::NodeConfig* config_;
  // config_ = new BT::NodeConfig();
  // auto blackboard = BT::Blackboard::create();
  // blackboard->set<std::chrono::milliseconds>(
  //     "bt_loop_duration",
  //     std::chrono::milliseconds(10));
  // communication_interfaces::msg::DrawerLeds expected_drawer_led;
  // expected_drawer_led.red = 5;
  // expected_drawer_led.green = 255;
  // expected_drawer_led.blue = 10;
  // expected_drawer_led.mode = 1;
  // expected_drawer_led.brightness = 128;
  // std::string led_tree_xml =
  //         R"(
  //         <root BTCPP_format="4" >
  //             <BehaviorTree ID="MainTree">
  //                 <ChangeLED
  //                     blue="10"
  //                     brightness="128"
  //                     drawer_address="{drawer}"
  //                     green="255"
  //                     led_topic="drawer_leds"
  //                     mode="1"
  //                     red="5"/>
  //             </BehaviorTree>
  //         </root>)";

  // blackboard->set<rclcpp::Node::SharedPtr>(
  //     "node",
  //     node);
  // auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
  // auto bt = bt_engine->createTreeFromText(led_tree_xml, blackboard);

  auto test_node = std::make_shared<BTTest>();
  test_node->configure();
  rclcpp::spin(test_node);
  rclcpp::shutdown();
  return 0;
}