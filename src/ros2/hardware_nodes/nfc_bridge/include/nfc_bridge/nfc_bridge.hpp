#ifndef HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_
#define HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_

#define STANDART_REPLAY_MESSAGE_SIZE 500
#define READER_INTEVALL              10
#define CHECK_ON_DB                  false

#include <string.h>

#include <memory>
#include <thread>

#include "communication_interfaces/action/create_user_nfc_tag.hpp"
#include "communication_interfaces/srv/write_nfc_tag.hpp"
#include "db_helper/postgresql_connector.hpp"
#include "elatec_api.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "serial_helper/serial_helper.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"

namespace nfc_bridge
{
  class NFCBridge : public rclcpp::Node
  {
   public:
    using CreateUser = communication_interfaces::action::CreateUserNfcTag;
    using GoalHandleCreateUser = rclcpp_action::ServerGoalHandle<CreateUser>;

    NFCBridge();
    ~NFCBridge();

    friend class TestNFCBridge;   // this class has full access to all private and protected parts of this class

   private:
    rclcpp_action::Server<CreateUser>::SharedPtr action_server_;
    int numReadings_;

    rclcpp::Service<communication_interfaces::srv::WriteNFCTag>::SharedPtr _write_nfc_tag_service;

    std::unique_ptr<serial_helper::ISerialHelper> _serial_connector;
    std::unique_ptr<db_helper::IDBHelper> _db_connector;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _nfc_key_publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _timer_subscriber;
    rclcpp_action::Server<CreateUser>::SharedPtr _create_user_server;

    void start_up_scanner();
    void shutdown_scanner();

    void reading_procedure();
    bool read_nfc_code(std::shared_ptr<std::string> scanned_key);
    // void createUser(const std::shared_ptr<GoalHandleCreateUser> goal_handle);

    void toggle_NFC_Reader_State(const std_msgs::msg::Bool::SharedPtr msg);
    void timer_start();
    void timer_stop();

    void write_nfc_tag(const std::shared_ptr<communication_interfaces::srv::WriteNFCTag::Request> request,
                       std::shared_ptr<communication_interfaces::srv::WriteNFCTag::Response> response);

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const CreateUser::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCreateUser> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleCreateUser> goal_handle);
    bool write_tag(int card_data);
    bool wait_for_tag();
    void send_nfc_cmd(std::string cmd);
  };

}   // namespace nfc_bridge
#endif   // HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_