#ifndef ELECTRICAL_DRAWER_HPP
#define ELECTRICAL_DRAWER_HPP
#include "i_drawer.hpp"
#include "motor.hpp"


// class ElectricalDrawer: public IDrawer{
//     public:
//         ElectricalDrawer(uint8_t id):Drawer(id){};

//         Motor* motor;

//           virtual void can_in(robast_can_msgs::CanMessage& msg){
//             if(can_message.get_id() == CAN_ID_ELECTRICAL_DRAWER_TASK){
//                     handle_electrical_drawer_task_msg(can_message);

//                     this->debug_prints_electric_drawer_task(can_message);
//                 }
//         }


//             void handle_electrical_drawer_task_msg(robast_can_msgs::CanMessage can_message){
//                 uint8_t drawer_id = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data();
//                 int speed = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_GOTO_POSITION).get_data();
//                 bool stallGuardEnable = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data() == CAN_DATA_ELECTRICAL_DRAWER_STALL_GUARD_ENABLED ? true : false;
//                 //ElectricalDrawer* drawer = &e_drawers.at(drawer_id);
             
//             }

//             virtual uint8_t get_drawer_id();

//         private:
//             uint8_t id;

//              robast_can_msgs::CanDb can_db_ = robast_can_msgs::CanDb();

//             void debug_prints_electric_drawer_task(robast_can_msgs::CanMessage can_message)
//             {
//                 Serial.print("Standard ID: ");
//                 Serial.print(this->rx_msg_id_, HEX);
//                 Serial.print(" rx_dlc: ");
//                 Serial.print(uint8_t(this->rx_msg_dlc_), DEC);
//                 Serial.print(" MODULE ID: ");
//                 Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
//                 Serial.print(" DRAWER ID: ");
//                 Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
//                 Serial.print(" GOTO POSITION: ");
//                 Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_GOTO_POSITION).get_data(), DEC);
//                 Serial.print(" SPEED MODE: ");
//                 Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_SPEED_MODE).get_data(), DEC);
//                 Serial.print(" STALL GUARD ENABLE: ");
//                 Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data(), DEC);
//             }
// };

#endif