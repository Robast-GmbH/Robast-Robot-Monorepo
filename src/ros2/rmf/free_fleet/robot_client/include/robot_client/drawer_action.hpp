#ifndef RMF__ROBOT_CLIENT__DRAWER_ACTION_HPP_
#define RMF__ROBOT_CLIENT__DRAWER_ACTION_HPP_

#include "action.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "drawer_status.hpp"
#include <iostream>
#include <map>
#include <chrono>


namespace rmf_robot_client
{
  class DrawerAction : public Action
  {
  private:

    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using StdMsgString= std_msgs::msg::String;
    using StdMsgBool= std_msgs::msg::Bool;
    using StdMsgInt = std_msgs::msg::Int64;

    bool is_edrawer;
    int module_id;
    int drawer_id;
    std::vector<u_int16_t> autorised_user;

    std::shared_ptr<std::map<std::string, DrawerStatus>> drawers;
    std::unique_ptr<DrawerStatus> selected_drawer;

    rclcpp::TimerBase::SharedPtr nfc_reading_timer;

    bool open_drawer(int module_id, int drawer_id);
    bool open_drawer_action(int target_module_id, int target_drawer_id);
    bool close_drawer(int module_id, int drawer_id);
    bool all_drawers_closed();
    void start_authentication_scan();
    void end_authentication_scan(bool successull);

    //subscriber
    rclcpp::Subscription<DrawerAddress>::SharedPtr drawer_status_subscriber_;
    rclcpp::Subscription<StdMsgInt>::SharedPtr authentication_subscriber_;
    
    // publisher
    rclcpp::Publisher<DrawerAddress>::SharedPtr trigger_open_drawer_publisher_;
    rclcpp::Publisher<DrawerAddress>::SharedPtr trigger_close_drawer_publisher_;
    rclcpp::Publisher<StdMsgBool>::SharedPtr nfc_on_off_publisher_;

  public:
    DrawerAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config, std::shared_ptr<std::map<std::string, DrawerStatus>> drawers, int drawer_int, int module_id, bool is_edrawer, std::vector<uint16_t> autorised_user);
    bool start(std::function<void(int)> next_action_callback);
    bool cancel();
    std::string get_type();
    bool receive_new_settings(std::string command, std::vector<std::string> value);
    void check_scant_user(const StdMsgInt &msg);
    //~DrawerAction();
  };
} // namespace rmf_robot_client

#endif // RMF__ROBOT_CLIENT__DRAWER_ACTION_HPP_