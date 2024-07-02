#ifndef HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_
#define HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_

#define STANDART_REPLAY_MESSAGE_SIZE 500
#define READER_INTEVALL              10
#define CHECK_ON_DB                  false

#include <string.h>
#include <memory>
#include <thread>

#include "communication_interfaces/action/create_user_nfc_tag.hpp"
#include "db_helper/postgresql_connector.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "serial_helper/serial_helper.h"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"
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

    std::unique_ptr<serial_helper::ISerialHelper> serial_connector_;
    std::unique_ptr<db_helper::IDBHelper> db_connector_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr authentication_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr timer_subscriber_;
    rclcpp_action::Server<CreateUser>::SharedPtr create_user_server_;

    void start_up_scanner();
    void shutdown_scanner();
    
    void reading_procedure();
    bool read_nfc_code(std::shared_ptr<std::string> scanned_key);
    void createUser(const std::shared_ptr<GoalHandleCreateUser> goal_handle);
    
    void control_timer(const std_msgs::msg::Bool::SharedPtr msg);
    void timer_start();
    void timer_stop();
    
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const CreateUser::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCreateUser> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleCreateUser> goal_handle);
  };

}   // namespace nfc_bridge
#endif   // HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_