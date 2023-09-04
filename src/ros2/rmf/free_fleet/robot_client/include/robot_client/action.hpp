#ifndef RMF__ROBOT_CLIENT__ACTION_HPP_
#define RMF__ROBOT_CLIENT__ACTION_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "fleet_interfaces/msg/free_fleet_data_task_state.hpp"

namespace rmf_robot_client
{
  class Action
  {private:
      using FreeFleetDataTaskInfo = fleet_interfaces::msg::FreeFleetDataTaskState;
      int task_id;
      int step;
      std::shared_ptr<rclcpp::Node> ros_node;
      std::map<std::string, std::string> config;
      //task_info_publisher

    public:
      Action(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config);
    //  ~Action();
      virtual bool start()=0;
      virtual bool cancel()=0;
      virtual bool receive_new_settings(std::string command, std::string value)=0;
      void publish_task_state(std::string status, std::string message, bool completed);

      int get_step();
      int get_task_id();
  };

} // namespace robot_client
#endif   // RMF__ROBOT_CLIENT__ACTION_HPP_