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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace rmf_robot_client
{
  class DrawerTask : public BaseTask
  {
   public:
    DrawerTask(int task_id,
               int step,
               std::shared_ptr<rclcpp::Node> ros_node,
               std::shared_ptr<std::map<std::string, DrawerState>> drawer_states,
               int drawer_id,
               int module_id,
               bool is_edrawer,
               std::vector<uint16_t> autorised_user);

    bool start(std::function<void(int)> next_task_callback) override;
    bool cancel();
    std::string get_type();
    bool receive_new_settings(std::string command, std::vector<std::string> value) override;
    void task_done(bool completted);

   private:
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using StdMsgBool = std_msgs::msg::Bool;

    bool is_e_drawer_;
    int module_id_;
    int drawer_id_;
    std::vector<u_int16_t> authorised_users_;

    int nfc_timeout_interval_;
    u_int16_t active_user_;

    std::shared_ptr<std::map<std::string, DrawerState>> drawers_;
    std::unique_ptr<DrawerState> selected_drawer_;
    rclcpp::TimerBase::SharedPtr nfc_timeout_timer_;

    void open_drawer(int module_id, int drawer_id);
    void open_drawer_task(int module_id, int drawer_id, bool is_edrawer);
    void close_drawer(int module_id, int drawer_id);
    void publish_close_drawer_status(int module_id, int drawer_id);
    std::string get_drawer_ref(int module_id, int drawer_id);
    bool check_user_permission(int user, std::vector<u_int16_t> authorised_user_list);
    bool all_drawers_closed();
    void start_authentication_scan();
    void end_authentication_scan();
    void check_scant_user(int user_id);
    void nfc_timeout();

    // publisher
    rclcpp::Publisher<DrawerAddress>::SharedPtr trigger_open_drawer_publisher_;
    rclcpp::Publisher<DrawerAddress>::SharedPtr trigger_open_e_drawer_publisher_;
    rclcpp::Publisher<DrawerAddress>::SharedPtr trigger_close_e_drawer_publisher_;
    rclcpp::Publisher<StdMsgBool>::SharedPtr nfc_on_off_publisher_;
  };
}   // namespace rmf_robot_client

#endif   // ROBOT_CLIENT__DRAWER_TASK_HPP_
