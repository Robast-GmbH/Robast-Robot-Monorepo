#ifndef RMF__ROBOT_CLIENT__DRAWER_ACTION_HPP_
#define RMF__ROBOT_CLIENT__DRAWER_ACTION_HPP_

#include "action.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rmf_robot_client
{
  class DrawerAction : public Action
  {
  private:

    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using StdMsgString= std_msgs::msg::String;
    using StdMsgBool= std_msgs::msg::Bool;

    bool is_edrawer;
    int module_id;
    int drawer_id;
    std::vector<std::string> redistricted_user;

    bool open_drawer();
    bool close_drawer();

    // publisher
    rclcpp::Subscription<DrawerAddress>::SharedPtr drawer_status_subscriber_;

    rclcpp::Publisher<DrawerAddress>::SharedPtr trigger_open_drawer_publisher_;
    rclcpp::Publisher<DrawerAddress>::SharedPtr trigger_close_drawer_publisher_;

  public:
    DrawerAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config, int drawer_int, int module_id, bool is_edrawer, std::vector<std::string> redistricted_user);
    bool start();
    bool cancel();
    bool receive_new_settings(std::string command, std::string value);
    //~DrawerAction();
  };
} // namespace rmf_robot_client

#endif // RMF__ROBOT_CLIENT__DRAWER_ACTION_HPP_