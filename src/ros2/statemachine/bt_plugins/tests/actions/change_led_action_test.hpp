#include "../catch_all.hpp"
#include <catch2/catch_amalgamated.hpp.hpp>
#include "../fakeit.hpp"
#include "bt_plugins/action/change_led_action.hpp"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/blackboard.h"


/*
* HOW TO RUN THIS TEST ON WINDOWS:
* - Go to directory libs/can/tests
* - Run the following commands:
* g++ -std=c++17 -c .\tests_main.cpp
* g++ -std=c++17 tests_main.o tests_robast_can_msgs.cpp ..\src\* -o test_executable -I .. -Wall -Wextra -Wuseless-cast -Wdouble-promotion -Wnull-dereference -Wpedantic -Wshadow -Wnon-virtual-dtor -Wlogical-op
* .\test_executable.exe
*/

namespace drawer_statemachine
{
    SCENARIO("Test class creation of CanSignal, CanMessage, CanDb and CanFrame", "[robast_can_msgs]")
    {

        GIVEN("A BT_engine")
        {
            //setup

            const std::vector<std::string> plugins = {
                "Hans",
                "drawer_open_request_action_bt_node",
            };
            static BT::NodeConfig* config_;
            config_ = new BT::NodeConfig();
            // Create the blackboard that will be shared by all of the nodes in the tree
            auto blackboard = BT::Blackboard::create();
            // Put items on the blackboard
            blackboard->set<std::chrono::milliseconds>(
                "bt_loop_duration",
                std::chrono::milliseconds(10));

            //hier brauch ich ros
            //create node
            rclcpp::Node::SharedPtr test_node = std::shared_ptr<rclcpp::Node>(rclcpp::Node("led_test_node"));
            std::string led_tree_xml =
                    R"(
                    <root BTCPP_format="4" >
                        <BehaviorTree ID="MainTree">
                            <ChangeLED blue="0"
                                brightness="128"
                                drawer_address="{drawer}"
                                green="255"
                                led_topic="drawer_leds"
                                mode="1"
                                red="0"/>
                        </BehaviorTree>
                    </root>)";

            

            /*
            testen:
            werden inuts aus bt gesetzt?
            stimmt der datentyp?
            wird der knoten richtig gebaut?



            change led testen:
            - plugin erstellen
            - brauch ich nen bt?
            - when - led config setzen
            - when - ticken
            - then - schauen ob die richtige config raus kommt

            */
            WHEN("The led plugin is created")
            {
                blackboard->set<rclcpp::Node::SharedPtr>(
                "node",
                std::shared_ptr<rclcpp::Node>(test_node));
                auto bt_engine = std::make_unique<drawer_statemachine::BehaviorTreeEngine>(plugins);
                
                auto bt = bt_engine->createTreeFromFile(led_tree_xml, blackboard);
                THEN("The input should be default")
                {}
                WHEN("A Tree gets created")
                    {
                        THEN("The created CanFrame class should encapsulate the data correctly")
                        {}
                    }
                WHEN("Creating the CanFrame class")
                {
                    THEN("The created CanFrame class should encapsulate the data correctly")
                    {}
                }
            }

        }
    }
}