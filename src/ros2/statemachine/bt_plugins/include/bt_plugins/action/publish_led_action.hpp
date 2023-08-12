#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__LEDPUBLISHER_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__LEDPUBLISHER_BT_NODES_H

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"

#include "bt_base_types/LED_base.hpp"

namespace statemachine
{
  /**
   * @brief A BT::ConditionNode that returns SUCCESS when goal is
   * updated on the blackboard and FAILURE otherwise
   */
  class LEDPublisherAction : public BT::StatefulActionNode
  {
  public:
    LEDPublisherAction(
        const std::string &name,
        const BT::NodeConfig &config);

    LEDPublisherAction() = delete;

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::vector<bt_base_types::LED>>(
              "", "address of the drawer thats used to execute the action"),
          BT::InputPort<communication_interfaces::msg::DrawerAddress>(
              "drawer_address", "address of the drawer thats used to execute the action"),
          BT::InputPort<std::string>(
              "led_topic",
              "/drawer_leds",
              "topic thats used to execute the action")};
    }

  protected:
    std::string topic_name_;
    BT::Blackboard::Ptr blackboard_;
    rclcpp::Publisher<communication_interfaces::msg::DrawerLeds>::SharedPtr led_publisher_;

  private:
    rclcpp::Node::SharedPtr _node;

    // rclcpp::CallbackGroup::SharedPtr _callback_group;
    // rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
  };
} // namespace statemachine
#endif