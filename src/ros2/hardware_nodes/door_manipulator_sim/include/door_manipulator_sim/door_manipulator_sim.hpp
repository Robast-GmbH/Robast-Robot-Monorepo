#ifndef DOOR_MANIPULATOR_GATE__DOOR_MANIPULATOR_SIM_HPP_
#define DOOR_MANIPULATOR_GATE__DOOR_MANIPULATOR_SIM_HPP_

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

using namespace std;
using namespace std::chrono_literals;

namespace door_manipulator_sim
{
    class DoorManipulatorSim: public rclcpp::Node
    {
    public:
        using norolem_msg = communication_interfaces::msg::NorelemStepperControl;
        DoorManipulatorSim();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<norolem_msg>::SharedPtr publisher_;

        
        void timer_callback();
        int getkey(void);
        void set_Motor(int motor_id, int speed);
        
    }; // namespace door_manipulator_sim

}

#endif  // DOOR_MANIPULATOR_SIM__DOOR_MANIPULATOR_SIM_HPP_