#ifndef BT_BASE_NODES_BT_SUB_INITIATOR__HEARTBEAT_TREE_INITIATOR_HPP
#define BT_BASE_NODES_BT_SUB_INITIATOR__HEARTBEAT_TREE_INITIATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "bt_base_nodes/bt_sub_base.hpp"
#include "std_msgs/msg/string.hpp"

namespace bt_base_nodes
{
  class HeartbeatTreeInitiator : public bt_base_nodes::BTSubBase<std_msgs::msg::String>
  {
   public:
    HeartbeatTreeInitiator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : BTSubBase(options)
    {
    }

   protected:
    void callbackRunBT(const std_msgs::msg::String::SharedPtr msg) override
    {
      BT::NodeStatus status = bt_.tickWhileRunning(std::chrono::milliseconds(tree_tick_time_));

      if (status != BT::NodeStatus::RUNNING)
      {
        bt_.haltTree();
        rclcpp::shutdown();
      }
    }

   private:
    std::string _id;
  };

}   // namespace bt_base_nodes

#endif   // BT_BASE_NODES_BT_SUB_INITIATOR__HEARTBEAT_TREE_INITIATOR_HPP