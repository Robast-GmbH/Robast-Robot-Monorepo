#include "robot_client/base_task.hpp"

namespace rmf_robot_client
{

  BaseTask::BaseTask(TaskId task_id, std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<TaskId> task_indicator)
      : task_id_(task_id), ros_node_(ros_node), task_indicator_(task_indicator)
  {
    task_info_publisher_ = ros_node->create_publisher<FleetDataTaskState>(
        ros_node->get_parameter("fleet_communication_task_info_topic").as_string(),
        QoSConfig::get_fleet_communication_qos());
  }

  int BaseTask::get_task_id() const
  {
    return task_id_.id;
  }

  int BaseTask::get_phase() const
  {
    return task_id_.phase;
  }

  void BaseTask ::assign_next_task(std::shared_ptr<BaseTask> next_task)
  {
    next_task_ = next_task;
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

  void BaseTask::publish_task_state(std::string status, std::string message, bool completed)
  {
    FleetDataTaskState task_state_msg;
    task_state_msg.task_id = task_id_.Tostring();
    task_state_msg.status = status;
    task_state_msg.status_message = message;
    task_state_msg.completed = completed;
    task_info_publisher_->publish(task_state_msg);
  }

  void BaseTask::start_next_phase()
  {
    if (next_task_ != nullptr)
    {
      if (task_id_ == *task_indicator_)
      {
        *task_indicator_ = next_task_->task_id_;
      }
      next_task_->start();
    }
    else
    {
      if (task_id_ == *task_indicator_)
      {
        task_indicator_->unset();
      }
    }
  }

}   // namespace rmf_robot_client