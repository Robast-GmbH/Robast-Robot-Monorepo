#ifndef DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__NFC_TO_DRAWER_ACTION_HPP
#define DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__NFC_TO_DRAWER_ACTION_HPP

#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace statemachine
{
  class NFCToDrawer : public BT::StatefulActionNode
  {
  public:
    NFCToDrawer(
        const std::string &name,
        const BT::NodeConfig &config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts()
    {
      return {};
    }

  protected:
    BT::Blackboard::Ptr blackboard_;

  private:
    rclcpp::Node::SharedPtr _node;
    std::map<std::string, communication_interfaces::msg::DrawerAddress> _nfc_key_to_DrawerAddress;
  };
} // namespace statemachine

#endif // DRAWER_STATEMACHINE__BT_PLUGINS__ACTION__NFC_TO_DRAWER_ACTION_HPP