#include "bt_plugins/condition/electric_drawer_status_condition.hpp"

#include <catch2/catch_test_macros.hpp>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "communication_interfaces/msg/electrical_drawer_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace test
{
  SCENARIO("A minimal tree gets created with just the ElectricDrawerStatusCondition plugin")
  {
    GIVEN("A minimal BT_engine with a one node electric drawer status condition tree")
    {
      const std::string nodename = "ElectricDrawerStatusCondition";
      const std::string topic_name = "/electrical_drawer_status";
      static bool is_initialized = false;
      if (!is_initialized)
      {
        rclcpp::init(0, nullptr);
        is_initialized = true;
      }

      const std::vector<std::string> plugins = {
        "electric_drawer_status_condition_bt_node",
      };
      static rclcpp::Node::SharedPtr node_electric_drawer_status =
        std::make_shared<rclcpp::Node>("test_electric_drawer_status");
      static BT::NodeConfig *config_;
      config_ = new BT::NodeConfig();
      auto blackboard = BT::Blackboard::create();
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
      blackboard->set<uint8_t>("target_value", 200);
      blackboard->set<bool>("use_stallguard", false);
      std::string electric_status_tree_xml =
        R"(
            <root BTCPP_format="4" >
                <BehaviorTree ID="MainTree">
                    <ElectricDrawerStatusCondition target_value="{target_value}" topic="/electrical_drawer_status" use_stallguard="{use_stallguard}"/>
                </BehaviorTree>
            </root>)";

      WHEN("The bt engine including the electric drawer status condition plugin is created")
      {
        auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
        blackboard->set<rclcpp::Node::SharedPtr>("node", node_electric_drawer_status);
        auto bt = bt_engine->createTreeFromText(electric_status_tree_xml, blackboard, "MainTree");

        THEN("A Subtree should exist")
        {
          REQUIRE(bt.subtrees[0]);

          AND_WHEN("the Tree exists")
          {
            THEN("the Tree should hold all added plugins, in this case ElectricDrawerStatusCondition")
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

            AND_THEN(
              "the ElectricDrawerStatusCondition node should transition correctly between states and evaluate the "
              "condition correctly")
            {
              auto result = bt.tickOnce();
              REQUIRE(result == BT::NodeStatus::RUNNING);

              auto publisher =
                node_electric_drawer_status->create_publisher<communication_interfaces::msg::ElectricalDrawerStatus>(
                  topic_name, 10);

              communication_interfaces::msg::ElectricalDrawerStatus status_msg;
              status_msg.position = 200;
              status_msg.is_stall_guard_triggered = false;

              publisher->publish(status_msg);
              rclcpp::spin_some(node_electric_drawer_status);

              result = bt.tickOnce();
              REQUIRE(result == BT::NodeStatus::SUCCESS);
            }

            AND_THEN("the ElectricDrawerStatusCondition node should return running if the condition is not satisfied")
            {
              auto result = bt.tickOnce();
              REQUIRE(result == BT::NodeStatus::RUNNING);

              communication_interfaces::msg::ElectricalDrawerStatus status_msg;
              status_msg.position = 194;
              status_msg.is_stall_guard_triggered = false;

              auto publisher =
                node_electric_drawer_status->create_publisher<communication_interfaces::msg::ElectricalDrawerStatus>(
                  topic_name, 10);
              publisher->publish(status_msg);
              rclcpp::spin_some(node_electric_drawer_status);

              result = bt.tickOnce();
              REQUIRE(result == BT::NodeStatus::RUNNING);
            }
          }
        }
      }
    }
  }
}   // namespace test