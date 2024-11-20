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
      this->declare_parameter("id", "not_set");
      _id = this->get_parameter("id").as_string();
    }

   protected:
    void callbackRunBT(const std_msgs::msg::String::SharedPtr msg) override
    {
      if (_id != msg->data)
      {
        // The way we currently spawn the heartbeat nodes:
        // In the HeartbeatTreeSpawner:
        // 1) Receives a heartbeat message
        // 2) Spawns a new HeartbeatTreeInitiator node with the id of the heartbeat message
        // 3) Sends a trigger message to the new HeartbeatTreeInitiator node
        // In the HeartbeatTreeInitiator:
        // 4) The new HeartbeatTreeInitiator node receives the trigger message and starts the tree

        // So in very rare cases it is possible that two HeartbeatTreeInitiator nodes are spawned (step 2)
        // before the trigger message is sent to the first one (step 3). So we need to check if the id
        // of the heartbeat message is the same as the id of the node before starting the tree.
        return;
      }

      blackboard_->set<std::string>("id", msg->data);
      BT::NodeStatus status = bt_.tickWhileRunning(std::chrono::milliseconds(tree_tick_time_));

      if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
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