#ifndef ROBOT_CLIENT__ACTION_HPP_
#define ROBOT_CLIENT__ACTION_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "fleet_interfaces/msg/free_fleet_data_task_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rmf_robot_client
{
  class Action
  {
   protected:
    using FreeFleetDataTaskState = fleet_interfaces::msg::FreeFleetDataTaskState;
    int task_id_;
    int step_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<FreeFleetDataTaskState>::SharedPtr task_info_publisher_;
    std::function<void(int)> finish_action;

   public:
    Action(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node);
    virtual bool start(std::function<void(int)> next_action_callback);
    virtual bool cancel() = 0;
    virtual bool receive_new_settings(std::string command, std::vector<std::string> value);
    virtual std::string get_type() = 0;
    void publish_task_state(std::string status, std::string message, bool completed);

    int get_step();
    int get_task_id();
  };

}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__ACTION_HPP_
