#include <catch2/catch_test_macros.hpp>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "bt_plugins/decorator/base_error_decorator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace decorator_tests
{
  SCENARIO("A minimal tree gets created with just the BaseErrorDecorator plugin.")
  {
    GIVEN("A minimal BT_engine with a one node tree")
    {
      std::string nodename = "BaseErrorDecorator";
      rclcpp::init(0, nullptr);

      const std::vector<std::string> plugins = {"base_error_decorator_node"};
      static rclcpp::Node::SharedPtr node_drawer_test = std::make_shared<rclcpp::Node>("test_condition");
      static BT::NodeConfig *config;
      config = new BT::NodeConfig();
      auto blackboard = BT::Blackboard::create();
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
      std::string decorator_tree_xml =
        R"(
                    <root BTCPP_format="4" >
                        <BehaviorTree ID="MainTree">
                            <BaseErrorDecorator topic="/robast_error">
                                <AlwaysSuccess/>
                            </BaseErrorDecorator>
                        </BehaviorTree>
                    </root>)";

      WHEN("The bt engine including the drawer status condition plugin is created")
      {
        blackboard->set<rclcpp::Node::SharedPtr>("node", node_drawer_test);
        auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
        auto bt = bt_engine->createTreeFromText(decorator_tree_xml, blackboard, "MainTree");
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
}   // namespace decorator_tests