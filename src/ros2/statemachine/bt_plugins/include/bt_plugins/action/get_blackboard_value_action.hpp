#ifndef BT_PLUGINS__ACTION__GET_BLACKBOARD_ENTRY_ACTION_HPP_
#define BT_PLUGINS__ACTION__GET_BLACKBOARD_ENTRY_ACTION_HPP_

#include <string>
#include <vector>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_plugins
{

  class GetBlackboardEntry : public BT::SyncActionNode
  {
  public:
    GetBlackboardEntry(
        const std::string &name,
        const BT::NodeConfig &config);

    GetBlackboardEntry() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("key"),
          BT::OutputPort<communication_interfaces::msg::DrawerAddress>("value")};
    }

  private:
    std::string _key;
    BT::Blackboard::Ptr _blackboard;
  };
}

#endif