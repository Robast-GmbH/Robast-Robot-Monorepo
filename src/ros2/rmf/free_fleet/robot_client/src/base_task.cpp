#include "robot_client/base_task.hpp"

namespace rmf_robot_client
{

  BaseTask::BaseTask(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node)
      : task_id_(task_id), step_(step), ros_node_(ros_node)
  {
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.avoid_ros_namespace_conventions(false);

    task_info_publisher_ = ros_node->create_publisher<FreeFleetDataTaskState>(
        ros_node_->get_parameter("fleet_communication_task_info_topic").as_string(), qos);
  }

  int BaseTask::get_task_id() const
  {
    return task_id_;
  }

  int BaseTask::get_step() const
  {
    return step_;
  }

  bool BaseTask::receive_new_settings(std::string command, std::vector<std::string> value)
  {
    if (command == "DrawerState")
    {
      publish_task_state("DrawerState", value[0], false);
      return true;
    }
    return false;
  }

  bool BaseTask::start(std::function<void(int)> next_task_callback)
  {
    finish_task_ = next_task_callback;
    return true;
  }

  void BaseTask::publish_task_state(std::string status, std::string message, bool completed)
  {
    FreeFleetDataTaskState task_state_msg;
    task_state_msg.task_id = task_id_ + "#" + step_;
    task_state_msg.status = status;
    task_state_msg.status_message = message;
    task_state_msg.completed = completed;
    task_info_publisher_->publish(task_state_msg);
  }

}   // namespace rmf_robot_client