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

    rclcpp::QoS qos_living_devices = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos_living_devices.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_living_devices.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_living_devices.avoid_ros_namespace_conventions(false);

    _heartbeat_sub = create_subscription<communication_interfaces::msg::Heartbeat>(
        "/heartbeat",
        qos_heartbeat_msgs,
        std::bind(&HeartbeatTreeSpawner::callback_heartbeat, this, std::placeholders::_1));

    _heartbeat_timeouts_sub = create_subscription<std_msgs::msg::String>(
        "/heartbeat_timeout",
        10,
        std::bind(&HeartbeatTreeSpawner::callback_heartbeat_timeout, this, std::placeholders::_1));

    _living_devices_pub = create_publisher<std_msgs::msg::String>("/living_devices", qos_living_devices);
  }

  void HeartbeatTreeSpawner::callback_heartbeat(const communication_interfaces::msg::Heartbeat::SharedPtr msg)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&HeartbeatTreeSpawner::handle_launching_of_new_heartbeat_trees, this, msg)}.detach();
  }

  void HeartbeatTreeSpawner::handle_launching_of_new_heartbeat_trees(
      const communication_interfaces::msg::Heartbeat::SharedPtr msg)
  {
    // Go through all living devices and check if the new device is already in the list
    if (_living_devices.find(msg->id) != _living_devices.end())
    {
      return;   // device already exists
    }
    else
    {
      _living_devices.insert(msg->id);
      publish_living_devices();
    }

    RCLCPP_INFO(get_logger(), "Creating new tree for device ID: %s", msg->id.c_str());

    _running_heartbeat_trees[msg->id] = std::move(std::jthread(
        [this, msg]()
        {
          try
          {
            const std::string setup_file = "/robast/" + std::string(std::getenv("ROS_DISTRO")) + "/setup.bash";
            const std::string command_to_run = "ros2 launch drawer_sm heartbeat_launch.py id:=" + msg->id;

            RCLCPP_INFO(
                get_logger(), "Sourcing: %s. And running command: %s", setup_file.c_str(), command_to_run.c_str());

            // Combine the conditional sourcing and command execution
            std::string shell_command = "[ -f " + setup_file + " ] && source " + setup_file + " && " + command_to_run +
                                        " || echo 'Setup file not found. Skipping setup.'";

            std::string bash_command = "bash -c \"" + shell_command + "\"";

            // Run the with std::system
            std::system(bash_command.c_str());
          }
          catch (const std::exception &e)
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to launch process: %s", e.what());
          }
        }));

    _heartbeat_tree_trigger[msg->id] = std::move(std::jthread(
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
        }));
  }

  void HeartbeatTreeSpawner::callback_heartbeat_timeout(const std_msgs::msg::String::SharedPtr msg)
  {
    _living_devices.erase(msg->data);
    RCLCPP_WARN(get_logger(), "Device with device ID %s timed out. Removed it from living devices.", msg->data.c_str());
    publish_living_devices();

    // Do the termination in a separate thread to avoid blocking the executor
    _terminate_tree_thread = std::move(std::jthread(
        [this, msg]()
        {
          terminate_tree(msg->data);
        }));
  }

  void HeartbeatTreeSpawner::terminate_tree(const std::string &id)
  {
    std::unordered_map<std::string, std::jthread>::iterator it_heartbeat_tree = _running_heartbeat_trees.find(id);
    if (it_heartbeat_tree != _running_heartbeat_trees.end())
    {
      RCLCPP_INFO(get_logger(), "Terminating tree for device ID: %s", id.c_str());
      it_heartbeat_tree->second.request_stop();
      it_heartbeat_tree->second.join();
      _running_heartbeat_trees.erase(it_heartbeat_tree);
    }

    // Although the trigger thread should have finished by now, we still check and join it
    std::unordered_map<std::string, std::jthread>::iterator it_trigger = _heartbeat_tree_trigger.find(id);
    if (it_trigger != _heartbeat_tree_trigger.end())
    {
      RCLCPP_INFO(get_logger(), "Terminating trigger thread for device ID: %s", id.c_str());
      it_trigger->second.request_stop();
      it_trigger->second.join();
      _heartbeat_tree_trigger.erase(it_trigger);
    }
  }

  void HeartbeatTreeSpawner::publish_living_devices()
  {
    std_msgs::msg::String msg;
    msg.data = "";

    uint16_t count = 0;
    // Copy _living_devices to a vector and sort it
    std::vector<std::string> devices(_living_devices.begin(), _living_devices.end());
    std::sort(devices.begin(), devices.end());

    for (const auto &device : devices)
    {
      if (count != 0 && count != devices.size())
      {
        msg.data += ",";
      }
      msg.data += device;
      count++;
    }

    _living_devices_pub->publish(msg);
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
