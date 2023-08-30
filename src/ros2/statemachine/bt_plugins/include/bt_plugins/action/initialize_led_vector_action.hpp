#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__INITIALIZE_LED_VECTOR_BT_NODES_H
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__INITIALIZE_LED_VECTOR_BT_NODES_H

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"

#include "bt_base_types/LED_base.hpp"

namespace bt_plugins
{
  /**
   * @brief A BT::ConditionNode that returns SUCCESS when goal is
   * updated on the blackboard and FAILURE otherwise
   */
  class InitLEDVector : public BT::SyncActionNode
  {
  public:
    InitLEDVector(
        const std::string &name,
        const BT::NodeConfig &config);

    InitLEDVector() = delete;

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
          BT::OutputPort<bt_base_types::LED>("led_vector")};
    } // namespace statemachine
  private:
    u_int8_t _size;
    std::vector<bt_base_types::LED> _led_vector;
  };
}
#endif