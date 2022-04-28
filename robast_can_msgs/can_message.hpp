#ifndef CAN_MESSAGE_HPP_
#define CAN_MESSAGE_HPP_

#include <string>
#include <vector>

namespace robast_can_msgs
{
    struct can_message
    {
        uint8_t id;
        std::string name;
        std::vector<can_signal> can_signals;
    };

    struct can_signal
    {
        std::string name;
        uint8_t offset; // "position" of the signal within a can_message
        uint8_t length; // number of bytes for this can_signal
    };
}

#endif /* CAN_MESSAGE_HPP_ */