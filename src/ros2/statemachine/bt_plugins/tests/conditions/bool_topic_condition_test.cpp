#include <catch2/catch_all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "bt_plugins/condition/bool_topic_condition.hpp"

namespace statemachine
{
  class BoolTopicConditionTest
  {
  protected:
    void SetUp()
    {
      rclcpp::init(0, nullptr);
      node = std::make_shared<rclcpp::Node>("test_node");
      BT::NodeConfig config;
      config.blackboard = BT::Blackboard::create();
      config.blackboard->set("node", node);
      bool_condition = std::make_shared<BoolTopicCondition>("test_bool_condition", config);
    }

    void TearDown()
    {
      rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<BoolTopicCondition> bool_condition;
  };

  TEST_CASE_METHOD(BoolTopicConditionTest, "TestComparator")
  {
    SetUp();
    std_msgs::msg::Bool msg;
    msg.data = true;
    REQUIRE(bool_condition->comparator(msg, true) == true);
    REQUIRE(bool_condition->comparator(msg, false) == false);
    TearDown();
  }

  TEST_CASE_METHOD(BoolTopicConditionTest, "TestCallbackTopicFeedback")
  {
    SetUp();
    auto msg = std::make_shared<std_msgs::msg::Bool>();
    msg->data = true;
    bool_condition->callbackTopicFeedback(msg);
    REQUIRE(bool_condition->tick() == BT::NodeStatus::SUCCESS);
    msg->data = false;
    bool_condition->callbackTopicFeedback(msg);
    REQUIRE(bool_condition->tick() == BT::NodeStatus::FAILURE);
    TearDown();
  }
}