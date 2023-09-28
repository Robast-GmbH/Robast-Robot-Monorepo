#ifndef RMF__ROBOT_CLIENT__DRAWER_ACTION_HPP_
#define RMF__ROBOT_CLIENT__DRAWER_ACTION_HPP_

#include "action.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "drawer_state.hpp"
#include <iostream>
#include <map>
#include <chrono>


namespace rmf_robot_client
{
  class DrawerAction : public Action
  {
  private:

    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using StdMsgBool= std_msgs::msg::Bool;

    bool is_e_drawer_;
    int module_id_;
    int drawer_id_;
    std::vector<u_int16_t> autorised_user_;

    int nfc_timeout_interval;
    u_int16_t active_user;


    std::shared_ptr<std::map<std::string, DrawerState>> drawers_;
    std::unique_ptr<DrawerState> selected_drawer_;
    rclcpp::TimerBase::SharedPtr nfc_timeout_timer_;

    void open_drawer(int module_id, int drawer_id);
    void open_drawer_action(int module_id, int drawer_id, bool is_edrawer);
    void close_drawer(int module_id, int drawer_id);
    void publish_close_drawer_status(int module_id, int drawer_id);
    std::string get_drawer_ref(int module_id, int drawer_id);
    bool check_user_permission(int user, std::vector<u_int16_t>authorised_user_list);
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

   public:
    DrawerAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<std::map<std::string, DrawerState>> drawers, int drawer_int, int module_id, bool is_edrawer, std::vector<uint16_t> autorised_user);
    bool start(std::function<void(int)> next_action_callback)override;
    bool cancel();
    std::string get_type();
    bool receive_new_settings(std::string command, std::vector<std::string> value) override;
    void action_done(bool completted);

    //~DrawerAction();
  };
} // namespace rmf_robot_client

#endif // RMF__ROBOT_CLIENT__DRAWER_ACTION_HPP_