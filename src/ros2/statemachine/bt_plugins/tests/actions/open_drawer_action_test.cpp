#include "bt_plugins/action/open_drawer_action.hpp"

#include <catch2/catch_test_macros.hpp>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "rclcpp/rclcpp.hpp"

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
      static BT::NodeConfig *config_;
      config_ = new BT::NodeConfig();
      auto blackboard = BT::Blackboard::create();
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
      communication_interfaces::msg::DrawerAddress used_drawer_address;
      used_drawer_address.module_id = 1;
      used_drawer_address.drawer_id = 1;
      blackboard->set<communication_interfaces::msg::DrawerAddress>("drawer_address", used_drawer_address);
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
        blackboard->set<rclcpp::Node::SharedPtr>("node", node_open_drawer);
        auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
        auto bt = bt_engine->createTreeFromText(led_tree_xml, blackboard, "MainTree");
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
        }
      }
    }
  }
}   // namespace test2