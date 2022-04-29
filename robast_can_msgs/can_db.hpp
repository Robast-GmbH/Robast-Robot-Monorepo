#ifndef CAN_DB_HPP_
#define CAN_DB_HPP_

#include "can_message.hpp"

namespace robast_can_msgs
{
    std::vector<can_message> can_db = {
        {0x01, "drawer_user_access", {
            {"drawer_id", 0, 24},
            {"open_drawer", 24, 1},
            {"LED_red", 25, 8},
            {"LED_green", 33, 8},
            {"LED_blue", 42, 8},
        }},
        {0x02, "drawer_feedback", {
            {"is_endstop_switch_pushed", 0, 1},
        }},
    }; 
}

#endif /* CAN_DB_HPP_ */