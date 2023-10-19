#ifndef RMF__ROBOT_CLIENT__ROBOT_CLIENT_HPP_
#define RMF__ROBOT_CLIENT__ROBOT_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"
#include <functional> 

#include "fleet_interfaces/msg/free_fleet_data_setting_request.hpp"
#include "fleet_interfaces/msg/free_fleet_data_create_nfc_request.hpp"
#include "fleet_interfaces/msg/free_fleet_data_drawer_request.hpp"
#include "fleet_interfaces/msg/free_fleet_data_destination_request.hpp"
#include "fleet_interfaces/msg/free_fleet_data_robot_state.hpp"

//#include "robotnik_msgs/msg/battery_status.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "action.hpp"
#include "nfc_action.hpp"
#include "drawer_action.hpp"
#include "navigation_action.hpp"
#include "drawer_state.hpp"

namespace rmf_robot_client
{
  class RobotClient : public rclcpp::Node
  {
    public:

          RobotClient();
          using FreeFleetDataSettingRequest = fleet_interfaces::msg::FreeFleetDataSettingRequest;
          using FreeFleetDataCreateNfcRequest = fleet_interfaces::msg::FreeFleetDataCreateNfcRequest;
          using FreeFleetDataDrawerRequest = fleet_interfaces::msg::FreeFleetDataDrawerRequest;
          using FreeFleetDataDestinationRequest = fleet_interfaces::msg::FreeFleetDataDestinationRequest;
          using FreeFleetDataRobotInfo = fleet_interfaces::msg::FreeFleetDataRobotState;
          
          using StdMsgBool = std_msgs::msg ::Bool;
          using StdMsgInt = std_msgs::msg::Int64;
          
          using DrawerStatus = communication_interfaces::msg::DrawerStatus;
          //using BatteryLevel = robotnik_msgs::msg::BatteryStatus;

        private:
          // Parameters
          std::string fleet_name;
          std::string robot_name;
          std::string robot_model;
          
          std::string behavior_tree;
          std::string map_frame_id;
          std::string robot_frame_id;

          //fleet_server
          rclcpp::Subscription<FreeFleetDataSettingRequest>::SharedPtr setting_subscriber_;
          rclcpp::Subscription<FreeFleetDataCreateNfcRequest>::SharedPtr write_nfc_card_request_subscriber_;
          rclcpp::Subscription<FreeFleetDataDrawerRequest>::SharedPtr drawer_request_subscriber_;
          rclcpp::Subscription<FreeFleetDataDestinationRequest>::SharedPtr navigation_request_subscriber_;
          
          //hardware
          //rclcpp::Subscription<BatteryLevel>::SharedPtr battery_status_sub_;
          rclcpp::Subscription<DrawerStatus>::SharedPtr drawer_status_subscriber_;
          rclcpp::Subscription<StdMsgInt>::SharedPtr authentication_subscriber_;
          
          //other
          rclcpp::Publisher<FreeFleetDataRobotInfo>::SharedPtr robot_info_publisher_;
          rclcpp::Publisher<StdMsgBool>::SharedPtr reset_simple_tree_publisher_;

          //Task
          int task_id;
          int current_step;
          float current_battery_level = -1;
          std::map<int, std::unique_ptr<Action>> task_sequence;
          rclcpp::TimerBase::SharedPtr publish_robot_info_timer_;
          int robot_info_inteval;
          int nfc_timeout_interval;

          //robot 
          std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
          std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
          double current_x;
          double current_y;
          double current_yaw;
          std::shared_ptr<std::map<std::string, DrawerState>> drawer_list;
          bool is_new_tf_error;

          void init_param();
          void start_receive_tasks();
          void start_update_robot_state();
          
          void receive_settings(const FreeFleetDataSettingRequest::SharedPtr msg);
          void receive_create_nfc_task(const FreeFleetDataCreateNfcRequest::ConstPtr msg);
          void receive_drawer_task(const FreeFleetDataDrawerRequest::ConstPtr msg);
          void receive_destination_task(const FreeFleetDataDestinationRequest::ConstPtr msg);
          
          void receive_drawer_status(const DrawerStatus::SharedPtr msg);
          void receive_authenticated_user(const StdMsgInt::SharedPtr msg);
          // void receive_battery_status(const BatteryLevel::ConstPtr msg);//ToDo: fix on the the bridge first

          bool prepare_new_action(std::string Task_def, std::string recipient_fleet, std::string recipient_robot, int &task_id, int &step);
          void end_current_task();
          void end_current_action(int step);
          void empty_task_sequence();
          void start_action(int step);
          void start_next_action();
          void publish_fleet_state();
          void update_robot_location();
     

          //support
          std::vector<std::string> split(std::string input_text, char delimiter);

  };
} // namespace robot_client
#endif   // RMF__ROBOT_CLIENT__ROBOT_CLIENT_HPP_
