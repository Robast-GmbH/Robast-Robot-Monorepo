#ifndef STATEMACHINE__BT_PLUGINS__ACTION__PUBLISH_STRING_TOPIC_HPP
#define STATEMACHINE__BT_PLUGINS__ACTION__PUBLISH_STRING_TOPIC_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"

namespace statemachine
{
  class PublishStringTopic : public BT::SyncActionNode
  {
   public:
    PublishStringTopic(const std::string &name, const BT::NodeConfig &config);

    PublishStringTopic() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("topic", "/topic", "topic thats used to publish the message"),
              BT::InputPort<std::string>("message", "message to be published")};
    }

   private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  };

}   // namespace statemachine

#endif   // STATEMACHINE__BT_PLUGINS__ACTION__PUBLISH_STRING_TOPIC_HPP