#include "bt_plugins/action/move_electric_drawer_action.hpp"

#include <catch2/catch_test_macros.hpp>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_task.hpp"
#include "rclcpp/rclcpp.hpp"

namespace test2
{
  SCENARIO("A minimal tree gets created with just the MoveElectricDrawer plugin")
  {
    GIVEN("A minimal BT_engine with a one node move drawer tree")
    {
      std::string nodename = "MoveElectricDrawer";
      static bool is_initialized = false;
      if (!is_initialized)
      {
        rclcpp::init(0, nullptr);
        is_initialized = true;
      }

      const std::vector<std::string> plugins = {
        "move_electric_drawer_action_bt_node",
        "get_blackboard_value_action_bt_node",
      };
      static rclcpp::Node::SharedPtr node_electric_drawer = std::make_shared<rclcpp::Node>("test_move_drawer");
      auto blackboard = BT::Blackboard::create();
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));

      communication_interfaces::msg::DrawerAddress used_drawer_address;
      used_drawer_address.module_id = 1;
      used_drawer_address.drawer_id = 1;
      blackboard->set<communication_interfaces::msg::DrawerAddress>("drawer_address", used_drawer_address);
      std::string electric_tree_xml =
        R"(
            <root BTCPP_format="4" >
                <BehaviorTree ID="MainTree">
                    <Sequence>
                        <GetBlackboardEntry key="drawer_address"
                        value="{drawer}"/>
                        <MoveElectricDrawer drawer_address="{drawer}"
                        move_electric_drawer_topic="move_electric_drawer" 
                        speed="200" 
                        target_position="200" 
                        stall_guard_value="0"/>
                    </Sequence>
                </BehaviorTree>
            </root>)";

      WHEN("The bt engine including the open drawer plugin is created")
      {
        auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
        blackboard->set<rclcpp::Node::SharedPtr>("node", node_electric_drawer);
        auto bt = bt_engine->createTreeFromText(electric_tree_xml, blackboard, "MainTree");
        THEN("A Subtree should exist")
        {
          REQUIRE(bt.subtrees[0]);
          WHEN("this Tree exists")
          {
            THEN("the Tree should hold all added plugins, in this case MoveElectricDrawer")
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

          AND_THEN("the MoveElectricDrawer node should publish the correct message")
          {
            bool callback_triggered = false;
            auto subscription = node_electric_drawer->create_subscription<communication_interfaces::msg::DrawerTask>(
              "move_electric_drawer",
              10,
              [&callback_triggered](const communication_interfaces::msg::DrawerTask::SharedPtr msg)
              {
                REQUIRE(msg->speed == 200);
                REQUIRE(msg->target_position == 200);
                REQUIRE(msg->stall_guard_value == 0);
                REQUIRE(msg->drawer_address.module_id == 1);
                REQUIRE(msg->drawer_address.drawer_id == 1);
                callback_triggered = true;
              });

            bt.tickOnce();                             // onStart
            auto result = bt.tickOnce();               // onRunning
            rclcpp::spin_some(node_electric_drawer);   // Process subscription callback
            REQUIRE(result == BT::NodeStatus::SUCCESS);
            REQUIRE(callback_triggered);
          }
        }
      }
    }
  }
}   // namespace test2