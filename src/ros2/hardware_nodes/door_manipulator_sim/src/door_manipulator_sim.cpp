#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include<iostream>

#include "rclcpp/rclcpp.hpp"
#include "communication_interfaces/msg/norelem_stepper_control.hpp"
using norolem_msg =communication_interfaces::msg::NorelemStepperControl; 
using namespace std;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class MinimalPublisher: public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<norolem_msg>("control_norelem_stepper", 10);//ToDo: Topic name
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        int ascii_value;
        char kp;
        kp = getkey();
        ascii_value = kp;
        
         switch (ascii_value)
         {
            case 113:
                set_Motor(1, 0);
                break;
            case 119:
                set_Motor(1, 1);
                break;
             case 101:
                set_Motor(1, 2);
                break;
             case 114:
                set_Motor(1, 3);
                break;
             case 116:
                 set_Motor(1, 4);
                 break;
             case 122:
                set_Motor(1, 5);
                break;
             case 117:
                set_Motor(1, 6);
                break;

                
             case 97:
                set_Motor(2, 0);
                break;
            case 115:
                set_Motor(2, 1);
                break;
             case 100:
                set_Motor(2, 2);
                break;
             case 102:
                set_Motor(2, 3);
                break;
             case 103:
                 set_Motor(2, 4);
                 break;
             case 104:
                set_Motor(2, 5);
                break;
             case 106:
                set_Motor(2, 6);
                break;
                
             case 121:
                set_Motor(3, 0);
                break;
            case 120:
                set_Motor(3, 1);
                break;
             case 99:
                set_Motor(3, 2);
                break;
             case 118:
                set_Motor(3, 3);
                break;
             case 98:
                 set_Motor(3, 4);
                 break;
             case 110:
                set_Motor(3, 5);
                break;
             case 109:
                set_Motor(3, 6);
                break;
                
             default:
                 set_Motor(0, 0);

         }
       
    }

    int getkey(void)
    {
        int ch;
        struct termios oldt;
        struct termios newt;
  
        tcgetattr(STDIN_FILENO, &oldt); /*store old settings */
        newt = oldt; /* copy old settings to new settings */
        newt.c_lflag &= ~(ICANON | ECHO); /* make one change to old settings in new settings */
  
        tcsetattr(STDIN_FILENO, TCSANOW, &newt); /*apply the new settings immediatly */
  
        ch = getchar(); /* standard getchar call */
  
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*reapply the old settings */
  
        return ch; /*return received char */    
    }

    void set_Motor(int motor_id, int speed)
    {
       auto message = norolem_msg();
       message.drawer_controller_id = 6;
       message.motor_id = motor_id;
       if (motor_id != 0)
       {

           switch (speed)
           {
           case 1:
               message.motor_e1 = true;
               break;
           case 2:
               message.motor_e2 = true;
               break;
           case 3:
               message.motor_e1 = true;
               message.motor_e2 = true;
               break;
           case 4:
               message.motor_e3 = true;
               break;
           case 5:
               message.motor_e1 = true;
               message.motor_e3 = true;
               break;
           case 6:
               message.motor_e2 = true;
               message.motor_e3 = true;
               break;
           case 7:
               message.motor_e1 = true;
               message.motor_e2 = true;
               message.motor_e3 = true;
               break;
           }
       }
           RCLCPP_INFO(this->get_logger(), "Moto: %i  Speed:  %i \n", motor_id, speed);
           publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<norolem_msg>::SharedPtr publisher_;
    size_t count_;
};




int main(int argc, char* argv[ ])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}