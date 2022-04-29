#ifndef CAN_MESSAGE_HPP_
#define CAN_MESSAGE_HPP_

#include <string>
#include <vector>

namespace robast_can_msgs
{
    struct can_signal
    {
        std::string name;
        uint8_t bit_start;
        uint8_t bit_length; // number of bits for this can_signal
    };

    struct can_message
    {
        uint16_t id;
        std::string name;
        std::vector<can_signal> can_signals;
    };

}

#endif /* CAN_MESSAGE_HPP_ */