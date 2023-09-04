#ifndef RMF__ROBOT_CLIENT__ROBOT_CLIENT_HPP_
#define RMF__ROBOT_CLIENT__ROBOT_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"

#include "fleet_interfaces/msg/free_fleet_data_setting_request.hpp"
#include "fleet_interfaces/msg/free_fleet_data_create_nfc_request.hpp"
#include "fleet_interfaces/msg/free_fleet_data_drawer_request.hpp"
#include "fleet_interfaces/msg/free_fleet_data_destination_request.hpp"
#include "fleet_interfaces/msg/free_fleet_data_robot_state.hpp"
// #include "fleet_interfaces/msg/free_fleet_data_task_state.hpp"

// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/bool.hpp"


// #include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "action.hpp"
#include "nfc_action.hpp"
#include "drawer_action.hpp"
#include "navigation_action.hpp"

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
          // using FreeFleetDataTaskInfo = fleet_interfaces::msg::FreeFleetDataTaskState;

          // using DrawerAddress = communication_interfaces::msg::DrawerAddress;

          // using StdMsgString= std_msgs::msg::String;
          // using StdMsgBool= std_msgs::msg::Bool;

        private:
          // Parameters
          std::string fleet_name;
          std::string robot_name;
          std::string robot_model;

          std::string robot_frame_id;
          std::string robot_odom;
          std::string behavior_tree;
          std::string move_base_server_name;
          std::string tf_goal_frame;
          std::string tf_start_frame;

          u_int8_t update_frequency;
          float patrol_break_frequency;
          float dds_domain;

          std::map<std::string, std::string> config;

          //fleet_server
          rclcpp::Subscription<FreeFleetDataSettingRequest>::SharedPtr setting_subscriber_;
          rclcpp::Subscription<FreeFleetDataCreateNfcRequest>::SharedPtr write_nfc_card_request_subscriber_;
          rclcpp::Subscription<FreeFleetDataDrawerRequest>::SharedPtr drawer_request_subscriber_;
          rclcpp::Subscription<FreeFleetDataDestinationRequest>::SharedPtr navigation_request_subscriber_;

          rclcpp::Publisher<FreeFleetDataRobotInfo>::SharedPtr robot_info_publisher_;
          // rclcpp::Publisher<FreeFleetDataTaskInfo>::SharedPtr task_info_publisher_;

          //NFC

          // rclcpp::Subscription<StdMsgString>::SharedPtr authenticated_user_;
          // rclcpp::Publisher<StdMsgBool>::SharedPtr controll_nfc_modules_publisher_;
          //self.create_nfc_action_client= ActionClient(self, CreateUserNfcTag,"/create_user")
        


          //Task
          int task_id;
          int current_step;
          std::map<int, std::unique_ptr<Action>> task_sequence;
          rclcpp::TimerBase::SharedPtr publish_robot_info_timer;

          //robot 
          std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
          std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
          double current_x;
          double current_y;
          double current_yaw;

          void init_param();
          void start_receive_tasks();
          void initialise_task_publisher();
          void receive_settings(const FreeFleetDataSettingRequest::SharedPtr msg);
          void receive_create_nfc_task(const FreeFleetDataCreateNfcRequest::ConstPtr msg);
          void receive_drawer_task(const FreeFleetDataDrawerRequest::ConstPtr msg);
          void receive_destination_task(const FreeFleetDataDestinationRequest::ConstPtr msg);
          bool prepare_new_action(std::string recipient_fleet, std::string recipient_robot, int task_id);
          void end_current_task();
          void empty_task_sequence();
          void start_next_action();
          void publish_fleet_state();
          void update_robot_location();
          void get_parameter_to_config(std::string parameter_name);

          //support
          std::vector<std::string> split(std::string input_text, char delimiter);
          double quaternion_to_yaw(double x, double y, double z, double w );
  };
} // namespace robot_client
#endif   // RMF__ROBOT_CLIENT__ROBOT_CLIENT_HPP_
