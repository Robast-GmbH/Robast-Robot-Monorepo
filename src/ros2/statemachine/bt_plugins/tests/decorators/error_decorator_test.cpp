#include <catch2/catch_test_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "bt_plugins/decorator/base_error_decorator.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/error_base_msg.hpp"
#include "error_utils/error_definitions.hpp"

namespace decorator_tests
{
  SCENARIO("A minimal tree gets created with just the BaseErrorDecorator plugin.")
  {
    GIVEN("A minimal BT_engine with a one node tree")
    {
      std::string nodename = "BaseErrorDecorator";
      static bool is_initialized = false;
      if (!is_initialized)
      {
        rclcpp::init(0, nullptr);
        is_initialized = true;
      }

      const std::vector<std::string> plugins = {"base_error_decorator_node"};
      static rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_condition");
      static BT::NodeConfig *config;
      config = new BT::NodeConfig();
      auto blackboard = BT::Blackboard::create();
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
      communication_interfaces::msg::DrawerAddress drawer_address;
      drawer_address.module_id = 1114113;
      drawer_address.drawer_id = 0;
      blackboard->set("drawer_address", drawer_address);

      std::string decorator_tree_xml =
        R"(
                    <root BTCPP_format="4" >
                        <BehaviorTree ID="MainTree">
                            <BaseErrorDecorator topic="/robast_error">
                                <AlwaysSuccess/>
                            </BaseErrorDecorator>
                        </BehaviorTree>
                    </root>)";

      WHEN("The bt engine including the BaseErrorDecorator plugin is created")
      {
        blackboard->set<rclcpp::Node::SharedPtr>("node", test_node);
        auto bt_engine = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
        auto bt = bt_engine->createTreeFromText(decorator_tree_xml, blackboard, "MainTree");
        THEN("A Subtree should exist")
        {
          REQUIRE(bt.subtrees[0]);
          WHEN("this Tree exists")
          {
            THEN("the Tree should contain the BaseErrorDecorator plugin node")
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

            AND_THEN("the BaseErrorDecorator node should return child status, which is success in this case")
            {
              auto result = bt.tickOnce();
              REQUIRE(result == BT::NodeStatus::SUCCESS);
            }

            AND_THEN("the BaseErrorDecorator node should handle error codes correctly")
            {
              // Simulate publishing an error message
              rclcpp::QoS qos(rclcpp::KeepLast(10));
              qos.transient_local().best_effort();
              auto publisher =
                test_node->create_publisher<communication_interfaces::msg::ErrorBaseMsg>("/robast_error", qos);
              communication_interfaces::msg::ErrorBaseMsg error_msg;
              error_msg.error_code = ERROR_CODES_TIMEOUT_DRAWER_NOT_OPENED;
              error_msg.error_data = error_utils::message_to_string(drawer_address);
              error_msg.error_description = "Drawer not opened in time";
              publisher->publish(error_msg);

              auto result = bt.tickOnce();

              REQUIRE(result == BT::NodeStatus::FAILURE);
            }
          }
        }
      }
    }
  }
}   // namespace decorator_tests
