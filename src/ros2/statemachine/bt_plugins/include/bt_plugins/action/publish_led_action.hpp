#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__LEDPUBLISHER_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__LEDPUBLISHER_BT_NODES_H

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "bt_base_types/led_base.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace statemachine
{
  class LEDPublisherAction : public BT::SyncActionNode
  {
   public:
    LEDPublisherAction(const std::string &name, const BT::NodeConfig &config);

    LEDPublisherAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::vector<bt_base_types::LED>>("leds",
                                                         "address of the drawer thats used to execute the action"),
          BT::InputPort<communication_interfaces::msg::DrawerAddress>(
              "drawer_address", "address of the drawer thats used to execute the action"),
          BT::InputPort<uint16_t>("fading_time_ms", 0, "fading time in ms"),
          BT::InputPort<std::string>("led_topic", "/drawer_leds", "topic thats used to execute the action"),
      };
    }

   protected:
    std::string topic_name_;

   private:
    BT::Blackboard::Ptr _blackboard;
    rclcpp::Publisher<communication_interfaces::msg::LedCmd>::SharedPtr _led_publisher;
    rclcpp::Node::SharedPtr _node;
  };
}   // namespace statemachine
#endif