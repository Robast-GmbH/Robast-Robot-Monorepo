#ifndef ROBOT_CLIENT__BASE_TASK_HPP_
#define ROBOT_CLIENT__BASE_TASK_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "fleet_interfaces/msg/fleet_data_task_state.hpp"
#include "qos_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "task_id.hpp"

namespace rmf_robot_client
{
  class BaseTask
  {
   public:
    BaseTask(TaskId task_id, std::shared_ptr<rclcpp::Node> ros_node);
    virtual void start() = 0;
    virtual bool cancel() = 0;
    virtual bool receive_new_settings(std::string command, std::vector<std::string> value);
    virtual std::string get_type() = 0;

    void assign_next_task(std::shared_ptr<BaseTask> next_task);

    int get_phase() const;
    int get_task_id() const;

   protected:
    using FleetDataTaskState = fleet_interfaces::msg::FleetDataTaskState;

    TaskId task_id_;
    std::shared_ptr<rclcpp::Node> ros_node_;

    void publish_task_state(std::string status, std::string message, bool is_completed);
    void start_next_phase();

   private:
    rclcpp::Publisher<FleetDataTaskState>::SharedPtr task_info_publisher_;
    std::shared_ptr<BaseTask> nextTask_;
  };

}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__BASE_TASK_HPP_
