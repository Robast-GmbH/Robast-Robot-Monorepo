#ifndef HARDWARE_NODES__NFC_GATE__NFC_GATE_HPP_
#define HARDWARE_NODES__NFC_GATE__NFC_GATE_HPP_

#define STANDART_REPLAY_MESSAGE_SIZE 500
#define READER_INTEVALL 500

#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// C library headers
#include <stdio.h>
#include <iostream>
#include <string.h>

// Linux headers for serial communication
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


#include "communication_interfaces/action/authenticate_user.hpp"
#include "communication_interfaces/srv/create_user_nfc_tag.hpp"
#include "nfc_gate/elatec_api.h"
#include "serial_helper/serial_helper.h"
#include "db_helper/postgresql_connector.hpp"

namespace robast
{

  class NFCGate : public rclcpp::Node
  {
  public:
    using  AuthenticateUser = communication_interfaces::action::AuthenticateUser;
    using GoalHandleAuthenticateUser = rclcpp_action::ServerGoalHandle<AuthenticateUser>;
    using CreateUser = communication_interfaces::srv::CreateUserNfcTag;

    NFCGate(std::string serial_port_path = "/dev/robast/robast_nfc");
    ~NFCGate();

    friend class TestNFCGate; // this class has full access to all private and protected parts of this class

  private:
    int numReadings_;

    serial_helper::ISerialHelper* serial_connector_;
    db_helper::IDBHelper* db_connector_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::shared_ptr<GoalHandleAuthenticateUser> timer_handle_;
    rclcpp_action::Server<AuthenticateUser>::SharedPtr user_authenticate_server_;
    rclcpp::Service<CreateUser>::SharedPtr create_user_server_;

    bool execute_scan(std::shared_ptr<std::string> recived_raw_data);
    bool scan_tag(std::shared_ptr<std::string> tag_data);

    void write_tag(const std::shared_ptr<CreateUser::Request> request, std::shared_ptr<CreateUser::Response> response);
    void start_up_scanner();
    void reader_procedure();
    void turn_off_scanner();
    bool checkUserTag(std::string scanned_key, std::shared_ptr<std::string> related_username);
  };



}  // namespace robast
#endif  // HARDWARE_NODES__NFC_GATE__NFC_GATE_HPP_