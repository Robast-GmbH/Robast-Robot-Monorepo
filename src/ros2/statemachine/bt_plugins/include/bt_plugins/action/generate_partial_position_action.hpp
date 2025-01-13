#ifndef STATEMACHINE__BT_PLUGINS__ACTION__GENERATE_PARTIAL_POSITION_ACTION_HPP
#define STATEMACHINE__BT_PLUGINS__ACTION__GENERATE_PARTIAL_POSITION_ACTION_HPP

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/action_node.h"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace statemachine
{
  constexpr uint8_t FIRST_LID_POSITION_OFFSET = 30;

  class GeneratePartialPosition : public BT::SyncActionNode
  {
   public:
    GeneratePartialPosition(const std::string &name, const BT::NodeConfig &config);

    GeneratePartialPosition() = delete;

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<communication_interfaces::msg::DrawerAddress>(
                  "drawer_address", "address of the tray we want to get the position for"),
              BT::InputPort<uint8_t>("first_tray_position", 0, "position of the first tray"),
              BT::InputPort<uint8_t>("last_tray_position", 0, "position of the last tray"),
              BT::InputPort<uint8_t>("tray_count", 0, "number of trays"),
              BT::OutputPort<uint8_t>("target_position")};
    }

   private:
    rclcpp::Node::SharedPtr _node;
  };

}   // namespace statemachine

#endif   // STATEMACHINE__BT_PLUGINS__ACTION__GENERATE_PARTIAL_POSITION_ACTION_HPP