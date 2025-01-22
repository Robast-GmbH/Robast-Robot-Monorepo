#include "bt_plugins/action/robast_error_pub_action.hpp"

#include <catch2/catch_all.hpp>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "communication_interfaces/msg/error_base_msg.hpp"
#include "error_utils/generic_error_converter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace test
{
  SCENARIO("A minimal tree gets created with just the RobastErrorPub plugin")
  {
    GIVEN("A minimal BT_engine with a one node robast error pub tree")
    {
      std::string nodename = "RobastErrorPub";
      static bool is_initialized = false;
      if (!is_initialized)
      {
        rclcpp::init(0, nullptr);
        is_initialized = true;
      }

      const std::vector<std::string> plugins = {
        "robast_error_pub_node",
      };
      static rclcpp::Node::SharedPtr ros_node = std::make_shared<rclcpp::Node>("ros_node");
      auto blackboard = BT::Blackboard::create();
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));

      uint16_t error_code = 50304;
      std::string id = "12345";
      blackboard->set("error_code", error_code);
      blackboard->set("id", id);

      std::string robast_error_pub_tree_xml =
        R"(
          <root BTCPP_format="4" >
              <BehaviorTree ID="MainTree">
                  <RobastErrorPub topic="/robast_error" error_code="{error_code}" error_data="{id}"/>
              </BehaviorTree>
          </root>)";

      WHEN("The bt engine including the robast error pub plugin is created")
      {
        auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
        blackboard->set<rclcpp::Node::SharedPtr>("node", ros_node);
        auto bt = bt_engine->createTreeFromText(robast_error_pub_tree_xml, blackboard, "MainTree");

        THEN("A subtree should exist")
        {
          REQUIRE(bt.subtrees.size() > 0);   // Ensure at least one subtree exists
          REQUIRE(bt.subtrees[0]);           // Check the first subtree exists

          WHEN("this Tree exists")
          {
            THEN("The tree should hold all added plugins, in this case RobastErrorPub")
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
            THEN("The tree should tick successfully and publish the correct error message")
            {
              bool callback_triggered = false;
              rclcpp::QoS qos_error_msgs = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
              qos_error_msgs.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
              qos_error_msgs.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
              qos_error_msgs.avoid_ros_namespace_conventions(false);

              auto subscription = ros_node->create_subscription<communication_interfaces::msg::ErrorBaseMsg>(
                "/robast_error",
                qos_error_msgs,
                [&callback_triggered, &error_code, &id](
                  const communication_interfaces::msg::ErrorBaseMsg::SharedPtr msg)
                {
                  REQUIRE(msg->error_code == error_code);
                  REQUIRE(msg->error_data == id);
                  callback_triggered = true;
                });

              auto result = bt.tickOnce();
              rclcpp::spin_some(ros_node);
              REQUIRE(result == BT::NodeStatus::SUCCESS);
              REQUIRE(callback_triggered);
            }
          }
        }
      }
    }
  }
}   // namespace test