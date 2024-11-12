#ifndef BT_PLUGINS_ACTION_ROBAST_ERROR_PUB_ACTION_HPP
#define BT_PLUGINS_ACTION_ROBAST_ERROR_PUB_ACTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "communication_interfaces/msg/error_base_msg.hpp"

namespace statemachine
{
  class RobastErrorPub : public BT::SyncActionNode
  {
   public:
    RobastErrorPub(const std::string &name, const BT::NodeConfig &config);

    RobastErrorPub() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("topic", "/robast_error"),
              BT::InputPort<uint16_t>("error_code", "Error code to be published"),
              BT::InputPort<std::string>(
                  "error_data",
                  "/robast_error",
                  "serialized data of a ros2 message, which contains more specific information about the error"),
              BT::InputPort<std::string>("error_description", "Some general description about the error")};
    }

   private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<communication_interfaces::msg::ErrorBaseMsg>::SharedPtr _error_publisher;
  };

}   // namespace statemachine

#endif   // BT_PLUGINS_ACTION_ROBAST_ERROR_PUB_ACTION_HPP