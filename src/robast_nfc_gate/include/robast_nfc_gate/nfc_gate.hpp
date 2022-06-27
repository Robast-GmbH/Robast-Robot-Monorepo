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

namespace robast_nfc_gate
{
  class NFCGate : public rclcpp::Node
  {

    public:
    using  AuthenticateUser= robast_ros2_msgs::action::AuthenticateUser;
    using GoalHandleAuthenticateUser = rclcpp_action::ServerGoalHandle<AuthenticateUser>;

    /**
    * @brief A constructor for robast_nfc_gate::NFCGate class
    */ 
    NFCGate();
    NFCGate(string serial_port_path );
   
    
    /**
    * @brief A destructor for robast_nfc_gate::NFCGate class
    */
    // ~NFCGate();
  

    private:
    string serial_path;
    int serial_port;


    
    rclcpp_action::Server<AuthenticateUser>::SharedPtr user_authenticate_server;

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID & uuid, shared_ptr<const AuthenticateUser::Goal> goal);
    rclcpp_action::CancelResponse cancel_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle); 
    void accepted_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle);
    void authenticate_user(const shared_ptr<GoalHandleAuthenticateUser> goal_handle);  
   
    void open_serial();
    void close_serial();
    
    string read_serial( );
    void write_serial( string msg );
    string send_command(string command );
    
    void scan(); 
};



}  // namespace robast_nfc_gate
#endif  // ROBAST_NFC_GATE__NFC_GATE_HPP_
