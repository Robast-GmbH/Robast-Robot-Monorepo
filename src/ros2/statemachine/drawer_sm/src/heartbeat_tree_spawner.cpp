#include "drawer_sm/heartbeat_tree_spawner.hpp"

namespace drawer_sm
{
  HeartbeatTreeSpawner::HeartbeatTreeSpawner() : Node("heartbeat_tree_spawner")
  {
    setup_subscriptions();
  }

  void HeartbeatTreeSpawner::setup_subscriptions()
  {
    rclcpp::QoS qos_heartbeat_msgs = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos_heartbeat_msgs.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_heartbeat_msgs.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_heartbeat_msgs.avoid_ros_namespace_conventions(false);

    heartbeat_sub_ = create_subscription<communication_interfaces::msg::Heartbeat>(
        "/heartbeat",
        qos_heartbeat_msgs,
        std::bind(&HeartbeatTreeSpawner::callback_heartbeat, this, std::placeholders::_1));
  }

  void HeartbeatTreeSpawner::callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&HeartbeatTreeSpawner::handle_launching_of_new_heartbeat_trees, this, msg)}.detach();
  }

  void HeartbeatTreeSpawner::handle_launching_of_new_heartbeat_trees(
      const communication_interfaces::msg::Heartbeat::SharedPtr msg)
  {
    std::vector<std::string> node_names = this->get_node_graph_interface()->get_node_names();

    const std::string target_node_name = "/heartbeat_tree_initiator_" + msg->id;
    if (std::find(node_names.begin(), node_names.end(), target_node_name) != node_names.end())
    {
      return;   // tree already exists
    }

    RCLCPP_INFO(get_logger(), "Creating new tree for %s", msg->id.c_str());

    std::thread(
        [this, msg]()
        {
          try
          {
            const std::string command = "ros2 launch drawer_sm heartbeat_launch.py id:=" + msg->id;
            std::system(command.c_str());
          }
          catch (const std::exception &e)
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to launch process: %s", e.what());
          }
        })
        .detach();

    std::thread(
        [this, msg]()
        {
          try
          {
            const std::string command =
                "ros2 topic pub /trigger_heartbeat_tree std_msgs/msg/String \"data: '" + msg->id + "'\" --once";
            std::system(command.c_str());
          }
          catch (const std::exception &e)
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to launch process: %s", e.what());
          }
        })
        .detach();
  }

}   // namespace drawer_sm

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drawer_sm::HeartbeatTreeSpawner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}