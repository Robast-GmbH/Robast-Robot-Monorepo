
#include "robot_client/action.hpp"

namespace rmf_robot_client
{
  
  Action::Action(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config)
  {
    this->task_id = task_id;
    this->step = step;
    this->ros_node = ros_node;
    this->config = config;
  }

  int Action::get_step()
  {
    return step;
  }

  int Action::get_task_id()
  {
    return task_id;
  }

  void Action::publish_task_state( std::string status, std::string  message, bool completed)
  {
    FreeFleetDataTaskInfo task_state_msg = FreeFleetDataTaskInfo();
    task_state_msg.task_id = task_id + "#" + step;
    task_state_msg.status = status;
    task_state_msg.status_message = message;
    task_state_msg.completed = completed;
    //task_info_publisher->publish(task_state_msg);
    // if (completed)
    // {
    //   start_next_action();
    // }
  }

}