#ifndef CAN_MESSAGE_HPP_
#define CAN_MESSAGE_HPP_

#include <string>
#include <vector>

namespace robast_can_msgs
{
    struct can_signal
    {
        std::string name;
        uint8_t bit_start_LSB;
        uint8_t bit_length; // number of bits for this can_signal
        uint64_t data;
    };

    struct can_message
    {
        uint16_t id;
        std::string name;
        std::vector<can_signal> can_signals;
    };

    can_message decode_can_message(can_message can_message, uint64_t can_data) {
        // TODO: Test this!
        for (int i = 0; i < can_message.can_signals.size(); i++) {
            can_message.can_signals[i].data = (can_data >> can_message.can_signals[i].bit_start_LSB) << (64 - can_message.can_signals[i].bit_length);
        }
        return can_message;
    }

}

#endif /* CAN_MESSAGE_HPP_ */