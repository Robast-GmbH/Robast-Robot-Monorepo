#ifndef ROBAST_NFC_GATE__NFC_GATE_HPP_
#define ROBAST_NFC_GATE__NFC_GATE_HPP_

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

#include "robast_ros2_msgs/action/authenticate_user.hpp"
#include "robast_ros2_msgs/srv/create_user_nfc_tag.hpp"
#include "robast_nfc_gate/elatec_api.h"
#include "include/robast_serial.h" //TODO: Fix that
   
namespace robast_nfc_gate
{ 

  
  class NFCGate : public rclcpp::Node
  {

    public:

    using  AuthenticateUser= robast_ros2_msgs::action::AuthenticateUser;
    using GoalHandleAuthenticateUser = rclcpp_action::ServerGoalHandle<AuthenticateUser>;
  
    using CreateUser= robast_ros2_msgs::srv::CreateUserNfcTag;
    /**
    * @brief A constructor for robast_nfc_gate::NFCGate class
    */ 
    NFCGate();
    NFCGate(string serial_port_path );
   
    private:
   
    int numReadings;
    //robast_serial::SerialHelper serial_connector;//= robast_serial::SerialHelper("");
    robast_serial::SerialHelper serial_connector = robast_serial::SerialHelper("/dev/serial/by-id/usb-Microchip_Technology__Inc._USBtin_A0211324-if00");
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp_action::Server<AuthenticateUser>::SharedPtr user_authenticate_server;
    rclcpp::Service<CreateUser>::SharedPtr create_user_server;

    

    rclcpp_action::GoalResponse auth_goal_callback(const rclcpp_action::GoalUUID & uuid, shared_ptr<const AuthenticateUser::Goal> goal);
    rclcpp_action::CancelResponse auth_cancel_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle); 
    void auth_accepted_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle);
    void auth_authenticate_user(const shared_ptr<GoalHandleAuthenticateUser> goal_handle);  
   
    void open_serial();
    void close_serial();
    
    string read_serial( );
    void write_serial( string msg );
    string send_command(string command );

    void scanTag( const std::shared_ptr<GoalHandleAuthenticateUser> goal_handle); 
    void writeTag(const std::shared_ptr<CreateUser::Request> request, std::shared_ptr<CreateUser::Response> response);
};



}  // namespace robast_nfc_gate
#endif  // ROBAST_NFC_GATE__NFC_GATE_HPP_
