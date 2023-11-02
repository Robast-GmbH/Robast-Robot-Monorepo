#ifndef ROBOT_CLIENT__DRAWER_TASK_HPP_
#define ROBOT_CLIENT__DRAWER_TASK_HPP_

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "base_task.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"
#include "drawer_state.hpp"
#include "qos_config.hpp"
#include "std_msgs/msg/bool.hpp"
#include "task_id.hpp"

namespace rmf_robot_client
{
  class DrawerTask : public BaseTask
  {
   public:
    DrawerTask(TaskId task_id,
               std::shared_ptr<rclcpp::Node> ros_node, 
               std::shared_ptr<TaskId> task_tracker,
               std::shared_ptr<std::map<std::string,DrawerState>> drawer_states,
               DrawerState used_drawer);

    void start() override;
    bool cancel();
    std::string get_type();
    bool receive_new_settings(std::string command, std::vector<std::string> value) override;
  

   private:
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using StdMsgBool = std_msgs::msg::Bool;

   // publisher
    rclcpp::Publisher<DrawerAddress>::SharedPtr _trigger_open_drawer_publisher;
    rclcpp::Publisher<DrawerAddress>::SharedPtr _trigger_open_e_drawer_publisher;
    rclcpp::Publisher<DrawerAddress>::SharedPtr _trigger_close_e_drawer_publisher;
    rclcpp::Publisher<StdMsgBool>::SharedPtr _nfc_on_off_publisher;

    rclcpp::TimerBase::SharedPtr _nfc_timeout_timer;    
    std::shared_ptr<std::map<std::string, DrawerState>> _drawers;
    std::unique_ptr<DrawerState> _selected_drawer;
    DrawerRef _drawer_to_be_opened_;
   
    int _nfc_timeout_interval;
    u_int16_t _active_user;

    void open_drawer(DrawerRef selected_drawer);
    void open_drawer_task(DrawerState selected_drawer);
    void close_drawer(DrawerRef selected_drawer);
    void publish_close_drawer_status(DrawerRef selected_drawer);
    bool check_user_permission(int user, std::vector<u_int16_t> authorised_user_list);
    bool all_drawers_closed();
    void start_authentication_scan();
    void end_authentication_scan();
    void check_scant_user(int user_id);
    void nfc_timeout();
    void task_done(bool completted);

 
  };
}   // namespace rmf_robot_client

#endif   // ROBOT_CLIENT__DRAWER_TASK_HPP_
