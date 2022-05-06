#include "can_message.h"

namespace robast_can_msgs
{
    CanMessage::CanMessage(uint32_t id_in, std::string name_in, std::vector<can_signal> can_signals_in)
    {
        id = id_in;
        name = name_in;
        can_signals_in = can_signals_in;
    }

    std::optional<CanMessage> decode_can_message(CAN_frame_t rx_frame, std::vector<CanMessage> can_db_messages)
    {
        std::vector<can_signal> can_signals;
        for (int j = 0; j < can_db_messages.size(); j++) {
            if (rx_frame.MsgID == can_db_messages[j].id) {
                uint64_t can_data = join_together_CAN_data_bytes(rx_frame);
                for (int i = 0; i < can_db_messages[j].can_signals.size(); i++) {
                    can_signals[i].bit_start = can_db_messages[j].can_signals[i].bit_start;
                    can_signals[i].bit_length = can_db_messages[j].can_signals[i].bit_length;
                    can_signals[i].data = (can_data << can_signals[i].bit_start) >> (64 - can_signals[i].bit_length);
                }
                return CanMessage(rx_frame.MsgID, can_db_messages[j].name, can_signals);
            }
        }
        return std::nullopt;
    }

    uint64_t join_together_CAN_data_bytes(CAN_frame_t rx_frame) {
        uint64_t can_data = 0;
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
            uint64_t can_byte = rx_frame.data.u8[i];
            can_data = (can_byte << (7-i)*8) | can_data;            
        };
        return can_data;
    }
}