
#include "door_manipulator_sim/door_manipulator_sim.hpp"

namespace door_manipulator_sim
{   
        DoorManipulatorSim::DoorManipulatorSim(): Node("door_manipulator_sim")
        {
            RCLCPP_INFO(this->get_logger(), "Speed 0: Motor1= R , Motor2 = F , Motor 3 = V");
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2));
            qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            qos.avoid_ros_namespace_conventions(false);
            
            publisher_ = this->create_publisher<norolem_msg>("control_norelem_stepper", qos);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&DoorManipulatorSim::timer_callback, this));
        }


        void DoorManipulatorSim::timer_callback()
        {
            int ascii_value;
            char kp;
            kp = getkey();
            ascii_value = kp;

            switch (ascii_value)
            {
                case 113:
                    set_Motor(1, -3);
                    break;
                case 119:
                    set_Motor(1, -2);
                    break;
                case 101:
                    set_Motor(1, -1);
                    break;
                case 114:
                    set_Motor(1, 0);
                    break;
                case 116:
                    set_Motor(1, 1);
                    break;
                case 122:
                    set_Motor(1, 2);
                    break;
                case 117:
                    set_Motor(1, 3);
                    break;


                case 97:
                    set_Motor(2, -3);
                    break;
                case 115:
                    set_Motor(2, -2);
                    break;
                case 100:
                    set_Motor(2, -1);
                    break;
                case 102:
                    set_Motor(2, 0);
                    break;
                case 103:
                    set_Motor(2, 1);
                    break;
                case 104:
                    set_Motor(2, 2);
                    break;
                case 106:
                    set_Motor(2, 3);
                    break;

                
                case 121:
                    set_Motor(3, -3);
                    break;
                case 120:
                    set_Motor(3, -2);
                    break;
                case 99:
                    set_Motor(3, -1);
                    break;
                case 118:
                    set_Motor(3, 0);
                    break;
                case 98:
                    set_Motor(3, 1);
                    break;
                case 110:
                    set_Motor(3, 2);
                    break;
                case 109:
                    set_Motor(3, 3);
                    break;

                default:
                    set_Motor(0, 0);
            }
        }

        int DoorManipulatorSim::getkey()
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

        void DoorManipulatorSim::set_Motor(int motor_id, int speed)
        {
            auto message = norolem_msg();
            message.drawer_controller_id = 6;
            message.motor_id = motor_id;
            if (motor_id != 0)
            {

                switch (speed)
                {
                case -3:
                    message.motor_e2 = true;
                    message.motor_e3 = true;
                    break;
                case -2:
                    message.motor_e3 = true;
                    break;
                case -1:
                    message.motor_e2 = true;
                    break;
                case -0:
                    message.motor_e1 = true;
                    break;
                case 1:
                    message.motor_e1 = true;
                    message.motor_e2 = true;
                    break;
                case 2:
                    message.motor_e1 = true;
                    message.motor_e3 = true;
                    
                    break;
                case 3:
                    message.motor_e1 = true;
                    message.motor_e2 = true;
                    message.motor_e3 = true;
                    break;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Moto: %i  Speed:  %i \n", motor_id, speed);
            RCLCPP_INFO(this->get_logger(), "M1 %i M2 %i M3 %i", message.motor_e1, message.motor_e2, message.motor_e3);
            publisher_->publish(message);
        }


}
