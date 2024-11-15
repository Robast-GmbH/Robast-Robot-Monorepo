#ifndef DRAWER_SM_HEARTBEAT_TREE_SPAWNER_HPP
#define DRAWER_SM_HEARTBEAT_TREE_SPAWNER_HPP

#include <boost/process.hpp>
#include <rclcpp/rclcpp.hpp>

#include "bt_base_nodes/bt_sub_initiator/heartbeat_tree_initiator.hpp"
#include "communication_interfaces/msg/heartbeat.hpp"

namespace drawer_sm
{
  class HeartbeatTreeSpawner : public rclcpp::Node
  {
   public:
    HeartbeatTreeSpawner();

   private:
    rclcpp::Subscription<communication_interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub_;

    void setup_subscriptions();

    void callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg);

    void handle_launching_of_new_heartbeat_trees(const communication_interfaces::msg::Heartbeat::SharedPtr msg);
  };

}   // namespace drawer_sm

#endif   // DRAWER_SM_HEARTBEAT_TREE_SPAWNER_HPP