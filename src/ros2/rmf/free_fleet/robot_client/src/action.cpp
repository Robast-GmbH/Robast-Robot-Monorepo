
#include "robot_client/action.hpp"

namespace rmf_robot_client
{
  
  Action::Action(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config)
  {
    this->task_id = task_id;
    this->step = step;
    this->ros_node = ros_node;
    this->config = config;
    
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.avoid_ros_namespace_conventions(false);
    
    task_info_publisher = ros_node->create_publisher<FreeFleetDataTaskInfo>(config["fleet_communication_task_info_topic"], qos);
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
    task_info_publisher->publish(task_state_msg);
  }

}