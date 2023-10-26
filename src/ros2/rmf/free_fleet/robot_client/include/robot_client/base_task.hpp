#ifndef ROBOT_CLIENT__BASE_TASK_HPP_
#define ROBOT_CLIENT__BASE_TASK_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "fleet_interfaces/msg/free_fleet_data_task_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "task_id.hpp"

namespace rmf_robot_client
{
  class BaseTask
  {
   public:
    BaseTask(TaskId task_id, std::shared_ptr<rclcpp::Node> ros_node);
    virtual bool start(std::function<void(int)> next_task_callback);
    virtual bool cancel() = 0;
    virtual bool receive_new_settings(std::string command, std::vector<std::string> value);
    virtual std::string get_type() = 0;
    void publish_task_state(std::string status, std::string message, bool is_completed);

    int get_step() const;
    int get_task_id() const;

   protected:
    using FreeFleetDataTaskState = fleet_interfaces::msg::FreeFleetDataTaskState;
    TaskId task_id_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<FreeFleetDataTaskState>::SharedPtr task_info_publisher_;
    std::function<void(int)> finish_task_;
  };

}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__BASE_TASK_HPP_
