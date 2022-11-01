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
using namespace std;

#ifdef GTEST
#define private public
#define protected public
#endif

#include "communication_interfaces/action/authenticate_user.hpp"
#include "communication_interfaces/srv/create_user_nfc_tag.hpp"
#include "nfc_gate/elatec_api.h"
#include "serial_helper/serial_helper.h" 

   
namespace robast
{ 

  
  class NFCGate : public rclcpp::Node
  {

    public:

      using  AuthenticateUser= communication_interfaces::action::AuthenticateUser;
      using GoalHandleAuthenticateUser = rclcpp_action::ServerGoalHandle<AuthenticateUser>;
      using CreateUser= communication_interfaces::srv::CreateUserNfcTag;

      /**
      * @brief A constructor for nfc_gate::NFCGate class
      */ 
      NFCGate();
      NFCGate(string serial_port_path );
      NFCGate(serial_helper::ISerialHelper *serial_connector );

      string execute_scan(std::vector<std::string> permission_keys, bool* found);
      string validate_key(string scanned_key, std::vector<std::string> allValidKeys, bool* found );
      void change_serial_helper(serial_helper::ISerialHelper* serial_connector );
      string scan_tag(bool* found); 
    
    private:
      int numReadings;
      bool debug;
      void reader_procedure();
      void start_up_scanner(); 
      void turn_off_scanner();
      serial_helper::ISerialHelper *serial_connector_;
      rclcpp::TimerBase::SharedPtr timer;
      shared_ptr<GoalHandleAuthenticateUser> timer_handle;
      rclcpp_action::Server<AuthenticateUser>::SharedPtr user_authenticate_server;
      rclcpp::Service<CreateUser>::SharedPtr create_user_server;

      rclcpp_action::GoalResponse auth_goal_callback(const rclcpp_action::GoalUUID & uuid, shared_ptr<const AuthenticateUser::Goal> goal);
      rclcpp_action::CancelResponse auth_cancel_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle); 
      void auth_accepted_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle);
      void auth_authenticate_user(const shared_ptr<GoalHandleAuthenticateUser> goal_handle);  
      void write_tag(const std::shared_ptr<CreateUser::Request> request, std::shared_ptr<CreateUser::Response> response);
      void mock_serial_connector();
};



}  // namespace robast
#endif  // NFC_GATE__NFC_GATE_HPP_
