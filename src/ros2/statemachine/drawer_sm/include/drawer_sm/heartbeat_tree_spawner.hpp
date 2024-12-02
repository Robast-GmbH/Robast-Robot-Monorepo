#ifndef DRAWER_SM_HEARTBEAT_TREE_SPAWNER_HPP
#define DRAWER_SM_HEARTBEAT_TREE_SPAWNER_HPP

#include <cstdlib>
#include <rclcpp/rclcpp.hpp>

#include "bt_base_nodes/bt_sub_initiator/heartbeat_tree_initiator.hpp"
#include "communication_interfaces/msg/heartbeat.hpp"
#include "std_msgs/msg/string.hpp"

namespace drawer_sm
{
  class HeartbeatTreeSpawner : public rclcpp::Node
  {
   public:
    HeartbeatTreeSpawner();

   private:
    rclcpp::Subscription<communication_interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_timeouts_sub_;

    void setup_subscriptions();

    void callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg);

    void handle_launching_of_new_heartbeat_trees(const communication_interfaces::msg::Heartbeat::SharedPtr msg);

    void callback_heartbeat_timeout(const std_msgs::msg::String::SharedPtr msg);

    std::unordered_set<std::string> _living_devices;
  };

}   // namespace drawer_sm

#endif   // DRAWER_SM_HEARTBEAT_TREE_SPAWNER_HPP