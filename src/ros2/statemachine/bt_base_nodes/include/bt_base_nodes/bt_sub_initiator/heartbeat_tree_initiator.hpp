#ifndef BT_BASE_NODES_BT_SUB_INITIATOR_HEARTBEAT_TREE_INITIATOR_HPP
#define BT_BASE_NODES_BT_SUB_INITIATOR_HEARTBEAT_TREE_INITIATOR_HPP

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
      blackboard_->set<std::string>("id", msg->data);
      bt_.tickWhileRunning(std::chrono::milliseconds(tree_tick_time_));
    }
  };

}   // namespace bt_base_nodes

#endif   // BT_BASE_NODES_BT_SUB_INITIATOR_HEARTBEAT_TREE_INITIATOR_HPP