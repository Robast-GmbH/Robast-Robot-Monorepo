#include <CAN.h>

#include "can_message.hpp"

namespace robast_can_msgs
{
    CanMessage::CanMessage(uint32_t id, std::string name, std::vector<can_signal> can_signals)
    {
        id = id;
        name = name;
        can_signals = can_signals;
    }

    CanMessage decode_can_message(CAN_frame_t rx_frame, std::vector<CanMessage> can_db, std::string name)
    {
        std::vector<can_signal> can_signals;
        for (int j = 0; j < can_db.size(); j++) {
            if (rx_frame.MsgID == can_db[j].id) {
                uint64_t can_data = join_together_CAN_data_bytes(rx_frame);
                for (int i = 0; i < can_signals.size(); i++) {
                    can_signals[i].data = (can_data >> can_signals[i].bit_start_LSB) << (64 - can_signals[i].bit_length);
                }
                return CanMessage(rx_frame.MsgID, name, can_signals);
            }
        }
        //TODO: Was returnen, wenn rx_frame nicht in db?
    }

    uint64_t join_together_CAN_data_bytes(CAN_frame_t rx_frame) {
        uint64_t can_data;
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
            printf("0x%02X ", rx_frame.data.u8[i]);
            // Join together the data of all CAN data bytes
            // TODO: Test this!
            uint64_t can_byte = rx_frame.data.u8[i];
            can_data = (can_byte << i*8) | can_data;
        };
        return can_data;
    }
}