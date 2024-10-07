#ifndef STATEMACHINE__BT_PLUGINS__ACTION__PUBLISH_STATE_FEEDBACK_H
#define STATEMACHINE__BT_PLUGINS__ACTION__PUBLISH_STATE_FEEDBACK_H
#include <string>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "communication_interfaces/msg/state_feedback.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace statemachine
{
  class PublishStateFeedback : public BT::SyncActionNode
  {
  public:
    PublishStateFeedback(
        const std::string &name,
        const BT::NodeConfig &config);

    PublishStateFeedback() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>(
              "state_feedback", "state feedback message to be published"),
          BT::InputPort<std::string>(
              "state_feedback_topic",
              "/state_feedback",
              "topic thats used to publish the state feedback message"),
          BT::InputPort<communication_interfaces::msg::DrawerAddress>(
              "drawer_address",
              "address of the drawer thats used to execute the action")};
    }

  private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<communication_interfaces::msg::StateFeedback>::SharedPtr _state_feedback_publisher;
  };
} // namespace statemachine

#endif // !STATEMACHINE__BT_PLUGINS__ACTION__PUBLISH_STATE_FEEDBACK_H
