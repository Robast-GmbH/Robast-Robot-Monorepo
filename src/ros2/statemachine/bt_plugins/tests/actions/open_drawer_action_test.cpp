#include <catch2/catch_all.hpp>
#include "bt_plugins/behavior_tree_engine.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/rclcpp.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "bt_plugins/action/open_drawer_action.hpp"




/*
* HOW TO RUN THIS TEST ON WINDOWS:
* - Go to directory libs/can/tests
* - Run the following commands:
* g++ -std=c++17 -c .\tests_main.cpp
* g++ -std=c++17 tests_main.o tests_robast_can_msgs.cpp ..\src\* -o test_executable -I .. -Wall -Wextra -Wuseless-cast -Wdouble-promotion -Wnull-dereference -Wpedantic -Wshadow -Wnon-virtual-dtor -Wlogical-op
* .\test_executable.exe
*/

namespace test2
{
    SCENARIO("A minimal tree gets created with just the OpenDrawer plugin")
    {
        GIVEN("A minimal BT_engine with a one node opendrawer tree")
        {
            std::string nodename = "OpenDrawer";
            rclcpp::init(0, nullptr);
            
            const std::vector<std::string> plugins = {
                "open_drawer_action_bt_node",
            };
            static rclcpp::Node::SharedPtr node_open_drawer = std::make_shared<rclcpp::Node>("test_open_drawer");
            static BT::NodeConfig* config_;
            config_ = new BT::NodeConfig();
            auto blackboard = BT::Blackboard::create();
            blackboard->set<std::chrono::milliseconds>(
                "bt_loop_duration",
                std::chrono::milliseconds(10));
            communication_interfaces::msg::DrawerAddress used_drawer_address;
            used_drawer_address.module_id = 1;
            used_drawer_address.drawer_id = 1;
            blackboard->set<communication_interfaces::msg::DrawerAddress>(
                "drawer_address",
                used_drawer_address);
            std::string led_tree_xml =
                    R"(
                    <root BTCPP_format="4" >
                        <BehaviorTree ID="MainTree">
                            <OpenDrawer drawer_address="{drawer}"
                            drawer_open_topic="/open_drawer"/>
                        </BehaviorTree>
                    </root>)";
            
            WHEN("The bt engine including the open drawer plugin is created")
            {
                blackboard->set<rclcpp::Node::SharedPtr>(
                    "node",
                    node_open_drawer);
                auto bt_engine = std::make_unique<drawer_statemachine::BehaviorTreeEngine>(plugins);
                auto bt = bt_engine->createTreeFromText(led_tree_xml, blackboard);
                THEN("A Subtree should exist")
                {
                    REQUIRE(bt.subtrees[0]);
                    WHEN("this Tree exists")
                    {
                        THEN("the Tree should hold all added plugins, in this case OpenDrawer")
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
                        }
                    }
                    //test cant be executed parallel to the test above. leeds to errors
                    // WHEN("The plugin is created")
                    // {
                    //     THEN("The LED config should be configured to the given values")
                    //     {
                            
                    //         auto iter = bt.subtrees[0]->nodes.begin();
                    //         for (; iter != bt.subtrees[0]->nodes.end(); iter++)
                    //         {
                    //             if ((*iter)->registrationName() == nodename)
                    //             {
                    //                 iter = bt.subtrees[0]->nodes.begin();
                    //                 break;
                    //             }
                    //         }
                    //         drawer_statemachine::ChangeLED* node = dynamic_cast<drawer_statemachine::ChangeLED*> ((*iter).get());
                    //         communication_interfaces::msg::DrawerLeds LED = node->getDrawerLED();
                    //         REQUIRE(LED.green == expected_drawer_led.green);
                    //         REQUIRE(LED.brightness == expected_drawer_led.brightness);
                    //         REQUIRE(LED.red == expected_drawer_led.red);
                    //         REQUIRE(LED.blue == expected_drawer_led.blue);
                    //         REQUIRE(LED.mode == expected_drawer_led.mode);
                    //     }
                    // }
                }
            }
        }
    }
}