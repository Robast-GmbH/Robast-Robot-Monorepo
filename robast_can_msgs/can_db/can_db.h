#ifndef CAN_DB_HPP_
#define CAN_DB_HPP_

#include "robast_can_msgs/can_message/can_message.h"

namespace robast_can_msgs
{
    class CanDb
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanDb class
             */
            CanDb() = default;

            // Vector that contains all the CAN messages stored in the CanDb
            const std::vector<CanMessage> can_messages = {
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
            };
    };
}

#endif /* CAN_DB_HPP_ */