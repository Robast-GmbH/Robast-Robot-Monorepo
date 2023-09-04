#ifndef RMF__ROBOT_CLIENT__NAVIGATION_ACTION_HPP_
#define RMF__ROBOT_CLIENT__NAVIGATION_ACTION_HPP_

#include "action.hpp"

namespace rmf_robot_client
{
  class NavigationAction : public Action
  {
    private:
      float x;
      float y;
      float yaw;

    public:
      NavigationAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config, float x, float y, float yaw);
      //~NavigationAction();

      bool start();
      bool cancel();
      bool receive_new_settings(std::string command, std::string value);
  };
}// namespace robot_client
#endif   // RMF__ROBOT_CLIENT__NAVIGATION_ACTION_HPP_