#include "can_message.hpp"

namespace robast_can_msgs
{
    CanMessage::CanMessage(uint16_t id, std::string name, std::vector<can_signal> can_signals)
    {
        id = id;
        name = name;
        can_signals = can_signals;
    }

    CanMessage decode_can_message(uint16_t id, std::string name, uint64_t can_data)
    {
        // TODO: Test this!
        std::vector<can_signal> can_signals;
        for (int i = 0; i < can_signals.size(); i++) {
            can_signals[i].data = (can_data >> can_signals[i].bit_start_LSB) << (64 - can_signals[i].bit_length);
        }
        return CanMessage(id, name, can_signals);
    }
}