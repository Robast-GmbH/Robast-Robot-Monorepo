#include "robot_client/navigation_action.hpp"

namespace rmf_robot_client
{
  NavigationAction::NavigationAction(int task_id, int step, std::shared_ptr<rclcpp::Node>  ros_node, std::map<std::string,std::string> config,  float x, float y, float yaw):Action(task_id, step, ros_node, config)
  {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
  }
  
  bool NavigationAction::start()
  {
    return true;
  }
  
  bool NavigationAction::cancel()
  {

  }
  
  bool NavigationAction::receive_new_settings(std::string command, std::string value)
  {

  }

}