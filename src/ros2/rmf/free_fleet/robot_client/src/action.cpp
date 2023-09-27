
#include "robot_client/action.hpp"

namespace rmf_robot_client
{
  
  Action::Action(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node)
  {
    this->task_id_ = task_id;
    this->step_ = step;
    this->ros_node_ = ros_node;
    
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.avoid_ros_namespace_conventions(false);
    
    task_info_publisher_ = ros_node->create_publisher<FreeFleetDataTaskInfo>(ros_node_->get_parameter("fleet_communication_task_info_topic").as_string(), qos);
  }

  int Action::get_step()
  {
    return step_;
  }

  int Action::get_task_id()
  {
    return task_id_;
  }

  bool Action::receive_new_settings(std::string command, std::vector<std::string> value)
  {
    if(command== "DrawerState")
    {
      publish_task_state("DrawerState", value[0], false);
      RCLCPP_INFO(ros_node_->get_logger(),"manual drawer state changed");
      return true;
    }
    return false;
  }

  void Action::publish_task_state( std::string status, std::string  message, bool completed)
  {
    FreeFleetDataTaskInfo task_state_msg = FreeFleetDataTaskInfo();
    task_state_msg.task_id = task_id_ + "#" + step_;
    task_state_msg.status = status;
    task_state_msg.status_message = message;
    task_state_msg.completed = completed;
    task_info_publisher_->publish(task_state_msg);
  }

}