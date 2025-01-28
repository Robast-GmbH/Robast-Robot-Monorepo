#include "bt_plugins/action/change_led_action.hpp"

#include <catch2/catch_test_macros.hpp>

#include "../test_helper.cpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"
#include "rclcpp/rclcpp.hpp"

namespace test
{
  SCENARIO(
    "A minimal tree gets created with just the ChangeLEDs plugin which has a given LED config, that should be used")
  {
    std::string nodename = "ChangeLED";
    GIVEN("A BT_engine")
    {
      rclcpp::init(0, nullptr);

      const std::vector<std::string> plugins = {
        "change_led_action_bt_node",
      };
      static rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test");
      static BT::NodeConfig *config_;
      config_ = new BT::NodeConfig();
      auto blackboard = BT::Blackboard::create();
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
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

      WHEN("The bt engine including the led plugin is created")
      {
        blackboard->set<rclcpp::Node::SharedPtr>("node", node);
        auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
        auto bt = bt_engine->createTreeFromText(led_tree_xml, blackboard, "MainTree");
        THEN("A Subtree should exist")
        {
          REQUIRE(bt.subtrees[0]);
          WHEN("this Tree exists")
          {
            THEN("the Tree should hold all added plugins, in this case ChangeLED")
            {
              auto iter = bt.subtrees[0]->nodes.begin();
              bool found = false;
              for (; iter != bt.subtrees[0]->nodes.end(); iter++)
              {
                if ((*iter)->registrationName() == nodename)
                {
                  iter = bt.subtrees[0]->nodes.begin();
                  found = true;
                  break;
                }
              }
              REQUIRE(found);
              statemachine::ChangeLED *node = dynamic_cast<statemachine::ChangeLED *>((*iter).get());
              communication_interfaces::msg::DrawerLeds LED = node->getDrawerLED();
              REQUIRE(LED.green == expected_drawer_led.green);
              REQUIRE(LED.brightness == expected_drawer_led.brightness);
              REQUIRE(LED.red == expected_drawer_led.red);
              REQUIRE(LED.blue == expected_drawer_led.blue);
              REQUIRE(LED.mode == expected_drawer_led.mode);
            }
          }
        }
      }
    }
  }
}   // namespace test