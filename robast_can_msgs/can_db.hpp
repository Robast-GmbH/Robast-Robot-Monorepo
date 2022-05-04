#ifndef CAN_DB_HPP_
#define CAN_DB_HPP_

#include "can_message.hpp"

namespace robast_can_msgs
{
    class CanDb
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanDb class
             */
            CanDb();

            // Vector that contains all the CAN messages stored in the CanDb
            const std::vector<CanMessage> can_messages;
    };
}

#endif /* CAN_DB_HPP_ */