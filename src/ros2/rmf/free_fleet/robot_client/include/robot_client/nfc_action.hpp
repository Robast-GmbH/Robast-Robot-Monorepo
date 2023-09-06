#ifndef RMF__ROBOT_CLIENT__NFC_ACTION_HPP_
#define RMF__ROBOT_CLIENT__NFC_ACTION_HPP_

#include "action.hpp"

namespace rmf_robot_client
{
  class NFCAction : public Action
  {
    private:
      int user_id;

    public:
      NFCAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config, int user_id);
      //~NFCAction();

      bool start(std::function<void(bool)> next_action_callback);
      bool cancel();
      std::string get_type();
      bool receive_new_settings(std::string command, std::string value);
  };
}// namespace robot_client
#endif   // RMF__ROBOT_CLIENT__NFC_ACTION_HPP_