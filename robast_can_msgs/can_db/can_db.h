#ifndef CAN_DB_HPP_
#define CAN_DB_HPP_

#include "robast_can_msgs/can_message/can_message.h"
#include "robast_can_msgs/can_db/can_db_defines.h"

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
                    CAN_ID_DRAWER_USER_ACCESS,
                    CAN_DLC_DRAWER_USER_ACCESS,
                    {
                        CanSignal(0, 24, 0),
                        CanSignal(24, 1, 0),
                        CanSignal(25, 8, 0),
                        CanSignal(33, 8, 0),
                        CanSignal(41, 8, 0),
                    }),
                CanMessage(
                    CAN_ID_DRAWER_FEEDBACK,
                    CAN_DLC_DRAWER_FEEDBACK,
                    {
                        CanSignal(0, 1, 0),
                        CanSignal(1, 1, 0),
                        CanSignal(2, 1, 0),
                        CanSignal(3, 1, 0)
                    })
            };
    };
}

#endif /* CAN_DB_HPP_ */