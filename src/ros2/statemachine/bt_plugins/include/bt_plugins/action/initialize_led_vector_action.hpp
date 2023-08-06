#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__CHANGELED_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__CHANGELED_BT_NODES_H

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"

namespace bt_plugins
{
  /**
   * @brief A BT::ConditionNode that returns SUCCESS when goal is
   * updated on the blackboard and FAILURE otherwise
   */
  class InitLEDVector : public BT::SyncActionNode
  {
  public:
    ChangeLED(
        const std::string &name,
        const BT::NodeConfig &config);

    ChangeLED() = delete;

    BT::NodeStatus tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<uint8_t>(
              "size", 128, "size"),
          BT::OutputPort<base_types::LED>("LEDs_colored")};
    } // namespace statemachine
#endif