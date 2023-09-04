#include "robot_client/nfc_action.hpp"

namespace rmf_robot_client
{

  NFCAction::NFCAction(int task_id, int step,std::shared_ptr<rclcpp::Node>  ros_node, std::map<std::string, std::string> config, int user_id) : Action(task_id, step, ros_node, config)
  {
    this->user_id = user_id;
  }

  bool NFCAction::start()
  {
    return true;
  }
  
  bool NFCAction::cancel()
  {

  }
  
  bool NFCAction::receive_new_settings(std::string command, std::string value)
  {

  }
}