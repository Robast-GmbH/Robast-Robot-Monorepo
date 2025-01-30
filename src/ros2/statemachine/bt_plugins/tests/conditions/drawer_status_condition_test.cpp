#include <catch2/catch_test_macros.hpp>
#include "bt_plugins/behavior_tree_engine.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_plugins/condition/drawer_status_condition.hpp"

namespace test2
{
    SCENARIO("A minimal tree gets created with just the DrawerStatusCondition plugin.")
    {
        GIVEN("A minimal BT_engine with a one node tree")
        {
            std::string nodename = "DrawerStatusCondition";
            rclcpp::init(0, nullptr);

            const std::vector<std::string> plugins = {
                "drawer_status_condition_bt_node",
            };
            static rclcpp::Node::SharedPtr node_drawer_test = std::make_shared<rclcpp::Node>("test_condition");
            static BT::NodeConfig *config;
            config = new BT::NodeConfig();
            auto blackboard = BT::Blackboard::create();
            blackboard->set<std::chrono::milliseconds>(
                "bt_loop_duration",
                std::chrono::milliseconds(10));
            std::string led_tree_xml =
                R"(
                    <root BTCPP_format="4" >
                        <BehaviorTree ID="MainTree">
                            <DrawerStatusCondition topic="/drawer_is_open"
                            target_value="false"/>
                        </BehaviorTree>
                    </root>)";

            WHEN("The bt engine including the drawer status condition plugin is created")
            {
                blackboard->set<rclcpp::Node::SharedPtr>(
                    "node",
                    node_drawer_test);
                auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
                auto bt = bt_engine->createTreeFromText(led_tree_xml, blackboard, "MainTree");
                THEN("A Subtree should exist")
                {
                    REQUIRE(bt.subtrees[0]);
                    WHEN("this Tree exists")
                    {
                        THEN("the Tree should contain the drawer status condition plugin node")
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
                }
            }
        }
    }
}