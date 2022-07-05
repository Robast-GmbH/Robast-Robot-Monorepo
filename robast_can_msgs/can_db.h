#ifndef CAN_DB_HPP_
#define CAN_DB_HPP_

#include "can_message.h"
#include "can_db_defines.h"

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
                        CanSignal(25, 1, 0),
                        CanSignal(26, 8, 0),
                        CanSignal(34, 8, 0),
                        CanSignal(42, 8, 0),
                    }),
                CanMessage(
                    CAN_ID_DRAWER_FEEDBACK,
                    CAN_DLC_DRAWER_FEEDBACK,
                    {
                        CanSignal(0, 24, 0),
                        CanSignal(24, 1, 0),
                        CanSignal(25, 1, 0),
                        CanSignal(26, 1, 0),
                        CanSignal(27, 1, 0)
                    })
            };
    };
}

#endif /* CAN_DB_HPP_ */