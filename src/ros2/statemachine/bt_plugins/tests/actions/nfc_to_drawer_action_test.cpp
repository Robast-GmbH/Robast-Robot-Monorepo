#include <catch2/catch_all.hpp>
#include "bt_plugins/action/nfc_to_drawer_action.hpp"
#include "bt_plugins/behavior_tree_engine.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/blackboard.h"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "rclcpp/rclcpp.hpp"

namespace test
{
  SCENARIO("A minimal tree gets created with just the NFCToDrawer plugin which has a given NFCToDrawer config, that should be used")
  {
    rclcpp::init(0, nullptr);
    std::string nodename = "NFCToDrawer";
    GIVEN("A BT_engine for the nfc test")
    {
      const std::vector<std::string> plugins = {
          "nfc_to_drawer_action_bt_node",
      };
      static rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test");
      static BT::NodeConfig *config_;
      config_ = new BT::NodeConfig();
      auto blackboard = BT::Blackboard::create();
      blackboard->set<std::chrono::milliseconds>(
          "bt_loop_duration",
          std::chrono::milliseconds(10));
      std::string nfc_tree_xml =
          R"(
                  <root BTCPP_format="4" >
                      <BehaviorTree ID="MainTree">
                          <NFCToDrawer/>
                      </BehaviorTree>
                  </root>)";

      WHEN("The bt engine including the nfc_to_drawer is created")
      {
        using DrawerAddress = communication_interfaces::msg::DrawerAddress;
        blackboard->set<rclcpp::Node::SharedPtr>(
            "node",
            node);
        std::map<std::string, DrawerAddress> nfc_dictionary;
        for (int i = 1; i <= 5; i++)
        {
          DrawerAddress drawer_address;
          drawer_address.module_id = 0;
          drawer_address.drawer_id = i;
          nfc_dictionary[std::to_string(i)] = drawer_address;
        }

        blackboard->set<std::map<std::string, DrawerAddress>>(
            "nfc_keys", nfc_dictionary);
        auto bt_engine1 = std::make_unique<statemachine::BehaviorTreeEngine>(plugins);
        auto bt = bt_engine1->createTreeFromText(nfc_tree_xml, blackboard, "MainTree");

        THEN("A Subtree should exist")
        {
          REQUIRE(bt.subtrees[0]);
          WHEN("this Tree exists")
          {
            THEN("the Tree should hold all added plugins, in this case NFCToDrawer and the drawer_address should be written to the blackboard after ticking")
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
              blackboard->set<std::string>("user_access_name", "1");
              bt.tickOnce();
              auto blackboard_drawer_address = blackboard->get<DrawerAddress>("drawer_address");
              REQUIRE(blackboard_drawer_address == nfc_dictionary["1"]);
            }
          }

          // geht hier nicht, weil das scenario geklont wird für parallele whens und damit 2 mal
          // versucht wird zwei mal rclcpp::init zu machen. //TODO Wenn ne andere lösung einfällt,
          // dann wieder zunzufügen
          // WHEN("the tree gets ticked with a key")
          // {
          //   blackboard->set<std::string>("user_access_name", "1");
          //   bt.tickOnce();

          //   THEN("the blackboard should contain the corresponding drawer address")
          //   {
          //     auto blackboard_drawer_address = blackboard->get<DrawerAddress>("drawer_address");
          //     REQUIRE(blackboard_drawer_address == nfc_dictionary["1"]);
          //   }
          // }
        }
      }
    }
  }
}