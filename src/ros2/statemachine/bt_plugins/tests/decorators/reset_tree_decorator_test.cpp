#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "bt_plugins/decorator/reset_tree_decorator.hpp"

namespace statemachine
{
  class TestResetDecorator : public ResetDecorator
  {
  public:
    using ResetDecorator::callbackResetFeedback; // Make method public for testing
    using ResetDecorator::ResetDecorator;        // Inherit constructor
  };

  class ResetDecoratorTest
  {
  protected:
    void SetUp()
    {
      rclcpp::init(0, nullptr);
      node = std::make_shared<rclcpp::Node>("test_node");
      BT::NodeConfig config;
      config.blackboard = BT::Blackboard::create();
      config.blackboard->set("node", node);
      reset_decorator = std::make_shared<TestResetDecorator>("test_reset_decorator", config);
    }

    void TearDown()
    {
      rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<TestResetDecorator> reset_decorator;
  };

  TEST_CASE_METHOD(ResetDecoratorTest, "TestCallbackResetFeedback")
  {
    SetUp();
    // Test the callbackResetFeedback function
    auto msg = std::make_shared<std_msgs::msg::Bool>();
    msg->data = true;
    reset_decorator->callbackResetFeedback(msg);
    REQUIRE(reset_decorator->status() == BT::NodeStatus::IDLE);
    TearDown();
  }
} // namespace statemachine