#ifndef ROBOT_CLIENT__ROBOT_CLIENT_HPP_
#define ROBOT_CLIENT__ROBOT_CLIENT_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fleet_interfaces/msg/fleet_data_create_nfc_request.hpp"
#include "fleet_interfaces/msg/fleet_data_destination_request.hpp"
#include "fleet_interfaces/msg/fleet_data_drawer_request.hpp"
#include "fleet_interfaces/msg/fleet_data_robot_state.hpp"
#include "fleet_interfaces/msg/fleet_data_setting_request.hpp"
#include "fleet_interfaces/msg/fleet_data_task_sequence_header_request.hpp"
#include "qos_config.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//  #include "robotnik_msgs/msg/battery_status.hpp"

#include "base_task.hpp"
#include "drawer_state.hpp"
#include "drawer_task.hpp"
#include "navigation_task.hpp"
#include "nfc_task.hpp"
#include "robot_pose.hpp"
#include "robot_ref.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "task_id.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rmf_robot_client
{
  class RobotClient : public rclcpp::Node
  {
   public:
    RobotClient();
    using FleetDataTaskSequenceHeaderRequest = fleet_interfaces::msg::FleetDataTaskSequenceHeaderRequest;
    using FleetDataSettingRequest = fleet_interfaces::msg::FleetDataSettingRequest;
    using FleetDataCreateNfcRequest = fleet_interfaces::msg::FleetDataCreateNfcRequest;
    using FleetDataDrawerRequest = fleet_interfaces::msg::FleetDataDrawerRequest;
    using FleetDataDestinationRequest = fleet_interfaces::msg::FleetDataDestinationRequest;
    using FleetDataRobotState = fleet_interfaces::msg::FleetDataRobotState;

    using StdMsgBool = std_msgs::msg::Bool;
    using StdMsgInt = std_msgs::msg::Int64;

    using DrawerStatus = communication_interfaces::msg::DrawerStatus;

    //  ToDo only works after the topic is bridged properly
    //  void RobotClient::receive_battery_status(const BatteryLevel::ConstPtr msg)
    // using BatteryLevel = robotnik_msgs::msg::BatteryStatus;

   private:
    // Parameters
    RobotRef _robot;
    std::string _robot_model;

    std::string _behavior_tree;
    std::string _map_frame_id;
    std::string _robot_frame_id;

    // fleet_server
    rclcpp::Subscription<FleetDataSettingRequest>::SharedPtr _setting_subscriber;
    rclcpp::Subscription<FleetDataCreateNfcRequest>::SharedPtr _write_nfc_card_request_subscriber;
    rclcpp::Subscription<FleetDataDrawerRequest>::SharedPtr _drawer_request_subscriber;
    rclcpp::Subscription<FleetDataDestinationRequest>::SharedPtr _navigation_request_subscriber;
    rclcpp::Subscription<FleetDataTaskSequenceHeaderRequest>::SharedPtr _task_sequence_request_subscriber;

    // hardware

    //  ToDo only works after the topic is bridged properly
    //  void RobotClient::receive_battery_status(const BatteryLevel::ConstPtr msg)
    // rclcpp::Subscription<BatteryLevel>::SharedPtr battery_status_sub_;
    rclcpp::Subscription<DrawerStatus>::SharedPtr _drawer_status_subscriber;
    rclcpp::Subscription<StdMsgInt>::SharedPtr _authentication_subscriber;

    // other
    rclcpp::Publisher<FleetDataRobotState>::SharedPtr _robot_info_publisher;
    rclcpp::Publisher<StdMsgBool>::SharedPtr _reset_simple_tree_publisher;

    // Task
    TaskId _current_task;
    std::map<int, std::shared_ptr<BaseTask>> _task_sequence;
    std::map<int, int> _task_sizes;
    std::thread _task_executer;
    std::mutex _receive_task_mutex;

    float _current_battery_level = -1;
    rclcpp::TimerBase::SharedPtr _publish_robot_info_timer;
    int _robot_info_inteval;
    int _nfc_timeout_interval;

    // robot
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    RobotPose _current_robot_pose = RobotPose(0, 0, 0);
    std::shared_ptr<std::map<std::string, DrawerState>> _drawer_list;
    bool _is_new_tf_error;

    void init_param();
    void start_receive_tasks();
    void start_update_robot_state();

    void receive_task_sequence_header(const FleetDataTaskSequenceHeaderRequest::ConstPtr msg);
    void receive_create_nfc_task(const FleetDataCreateNfcRequest::ConstPtr msg);
    void receive_drawer_task(const FleetDataDrawerRequest::ConstPtr msg);
    void receive_destination_task(const FleetDataDestinationRequest::ConstPtr msg);
    void receive_settings(const FleetDataSettingRequest::SharedPtr msg);

    void receive_drawer_status(const DrawerStatus::SharedPtr msg);
    void receive_authenticated_user(const StdMsgInt::SharedPtr msg);

    //  TODO @Torben only works after the topic is bridged properly
    //  void RobotClient::receive_battery_status(const BatteryLevel::ConstPtr msg)
    // void receive_battery_status(const BatteryLevel::ConstPtr msg);

    bool task_validation(std::string task_def, RobotRef robot, std::unique_ptr<TaskId> task_id);

    bool task_extension_validaton(TaskId task_id);
    void empty_task_sequence();
    bool start_task();
    bool link_tasks();

    void publish_fleet_state();
    void update_robot_location();
  };
}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__ROBOT_CLIENT_HPP_
