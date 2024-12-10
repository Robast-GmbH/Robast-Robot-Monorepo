#ifndef DRAWER_SM_HEARTBEAT_TREE_SPAWNER_HPP
#define DRAWER_SM_HEARTBEAT_TREE_SPAWNER_HPP

#include <boost/process.hpp>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

#include "bt_base_nodes/bt_sub_initiator/heartbeat_tree_initiator.hpp"
#include "communication_interfaces/msg/heartbeat.hpp"
#include "std_msgs/msg/string.hpp"

namespace drawer_sm
{
  constexpr uint8_t TIMEOUT_TREE_TERMINATION_IN_SEC = 2;

  class HeartbeatTreeSpawner : public rclcpp::Node
  {
   public:
    HeartbeatTreeSpawner();

   private:
    rclcpp::Subscription<communication_interfaces::msg::Heartbeat>::SharedPtr _heartbeat_sub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _heartbeat_timeouts_sub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _living_devices_pub;

    void setup_subscriptions();

    void callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg);

    void handle_launching_of_new_heartbeat_trees(const communication_interfaces::msg::Heartbeat::SharedPtr msg);

    void callback_heartbeat_timeout(const std_msgs::msg::String::SharedPtr msg);

    void publish_living_devices();

    void terminate_tree(const std::string &id);

    std::unordered_set<std::string> _living_devices;

    std::unordered_map<std::string, boost::process::child> _child_processes;
  };

}   // namespace drawer_sm

#endif   // DRAWER_SM_HEARTBEAT_TREE_SPAWNER_HPP
