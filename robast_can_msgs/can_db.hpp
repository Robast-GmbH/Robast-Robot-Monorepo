#ifndef CAN_DB_HPP_
#define CAN_DB_HPP_

#include "robast_can_msgs/can_message.hpp"

namespace robast_can_msgs
{
    std::vector<can_message> can_db = {
        {0x01, "drawer_user_access", {
            {"drawer_id", 0, 2}
            {"open_lock", 2, 1},
            {"LED_red", 3, 1},
            {"LED_green", 4, 1},
            {"LED_blue", 5, 1},
        }},
    }; 
}

#endif /* CAN_DB_HPP_ */