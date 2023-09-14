#ifndef RMF__ROBOT_CLIENT__ACTION_HPP_
#define RMF__ROBOT_CLIENT__ACTION_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "fleet_interfaces/msg/free_fleet_data_task_state.hpp"
#include <functional>

namespace rmf_robot_client
{
  class Action
  {
    protected: 
      using FreeFleetDataTaskInfo = fleet_interfaces::msg::FreeFleetDataTaskState;
      int task_id;
      int step;
      std::shared_ptr<rclcpp::Node> ros_node;
      std::map<std::string, std::string> config;
      rclcpp::Publisher<FreeFleetDataTaskInfo>::SharedPtr task_info_publisher;
      std::function<void(bool)> finish_action;

    public:
      Action(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config);
    //  ~Action();
      virtual bool start(std::function<void(int)> next_action_callback)=0;
      virtual bool cancel()=0;
      virtual bool receive_new_settings(std::string command, std::vector<std::string> value)=0;
      virtual std::string get_type() = 0;
      void publish_task_state(std::string status, std::string message, bool completed);

      int get_step();
      int get_task_id();
  };

} // namespace robot_client
#endif   // RMF__ROBOT_CLIENT__ACTION_HPP_