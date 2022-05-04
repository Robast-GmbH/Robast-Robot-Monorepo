#include "can_db.hpp"

namespace robast_can_msgs
{
    CanDb::CanDb()
    {
        can_messages = {
            CanMessage(
            0x01,
            "drawer_user_access",
            {
                {"drawer_id", 0, 24, 0},
                {"open_drawer", 24, 1, 0},
                {"LED_red", 25, 8, 0},
                {"LED_green", 33, 8, 0},
                {"LED_blue", 41, 8, 0},
            }),
            CanMessage(
            0x02,
            "drawer_feedback",
            {
                {"is_endstop_switch_pushed", 0, 1},
            })
        }            
    }
}