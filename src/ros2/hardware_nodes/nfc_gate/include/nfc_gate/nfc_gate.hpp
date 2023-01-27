#ifndef NFC_GATE__NFC_GATE_HPP_
#define NFC_GATE__NFC_GATE_HPP_

#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
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
#include "db_helper/postgresql_connector.h"

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
    int numReadings;

    serial_helper::ISerialHelper* serial_connector_;
    db_helper::IDBHelper* db_conncetor_;
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<GoalHandleAuthenticateUser> timer_handle;
    rclcpp_action::Server<AuthenticateUser>::SharedPtr user_authenticate_server;
    rclcpp::Service<CreateUser>::SharedPtr create_user_server;

    rclcpp_action::GoalResponse auth_goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const AuthenticateUser::Goal> goal);
    rclcpp_action::CancelResponse auth_cancel_callback(const std::shared_ptr<GoalHandleAuthenticateUser> goal_handle);
    void auth_accepted_callback(const std::shared_ptr<GoalHandleAuthenticateUser> goal_handle);
    void auth_authenticate_user(const std::shared_ptr<GoalHandleAuthenticateUser> goal_handle);

    std::string execute_scan(std::vector<std::string> permission_users, bool* found);
    std::string validate_key(std::string scanned_key, std::vector<std::string> allValidUser, bool* found);
    std::string scan_tag(bool* found);

    void write_tag(const std::shared_ptr<CreateUser::Request> request, std::shared_ptr<CreateUser::Response> response);
    void start_up_scanner();
    void reader_procedure();
    void turn_off_scanner();
  };



}  // namespace robast
#endif  // NFC_GATE__NFC_GATE_HPP_