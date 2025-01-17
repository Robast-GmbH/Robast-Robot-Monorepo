#include "bt_plugins/action/generate_partial_position_action.hpp"

#include <catch2/catch_all.hpp>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "rclcpp/rclcpp.hpp"

namespace test
{
  SCENARIO("A minimal tree gets created with just the GeneratePartialPosition plugin")
  {
    GIVEN("A minimal BT_engine with a one node generate partial position tree")
    {
      std::string nodename = "GeneratePartialPosition";
      static bool is_initialized = false;
      if (!is_initialized)
      {
        rclcpp::init(0, nullptr);
        is_initialized = true;
      }

      const std::vector<std::string> plugins = {
          "generate_partial_position_action_node",
      };
      auto node_generate_position = std::make_shared<rclcpp::Node>("test_generate_position");
      auto blackboard = BT::Blackboard::create();
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));

      communication_interfaces::msg::DrawerAddress used_drawer_address;
      used_drawer_address.module_id = 1;
      used_drawer_address.drawer_id = 2;
      blackboard->set("drawer_address", used_drawer_address);
      blackboard->set("front_offset", static_cast<uint8_t>(54));
      blackboard->set("tray_count", static_cast<uint8_t>(4));

      std::string generate_position_tree_xml =
          R"(
          <root BTCPP_format="4" >
              <BehaviorTree ID="MainTree">
                  <GeneratePartialPosition drawer_address="{drawer_address}"
                  front_offset="{front_offset}"
                  tray_count="{tray_count}"
                  target_position="{target_position}"/>
              </BehaviorTree>
          </root>)";

      WHEN("The bt engine including the generate partial position plugin is created")
      {
        auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
        blackboard->set("node", node_generate_position);
        auto bt = bt_engine->createTreeFromText(generate_position_tree_xml, blackboard, "MainTree");

        THEN("A subtree should exist")
        {
          REQUIRE(bt.subtrees.size() > 0);   // Ensure at least one subtree exists
          REQUIRE(bt.subtrees[0]);           // Check the first subtree exists

          WHEN("this Tree exists")
          {
            THEN("The tree should hold all added plugins, in this case GeneratePartialPosition")
            {
              bool found = std::ranges::any_of(bt.subtrees[0]->nodes,
                                               [&nodename](const auto& node)
                                               {
                                                 return node->registrationName() == nodename;
                                               });

              REQUIRE(found);
            }
          }

          WHEN("The tree is ticked")
          {
            THEN("The tree should tick successfully and set the correct target position")
            {
              auto result = bt.tickOnce();
              REQUIRE(result == BT::NodeStatus::SUCCESS);

              uint8_t target_position;
              blackboard->get("target_position", target_position);
              REQUIRE(target_position == 204);
            }
          }
        }
      }
    }
  }
}   // namespace test
