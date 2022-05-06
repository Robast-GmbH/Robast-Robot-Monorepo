#include "can_message.h"

namespace robast_can_msgs
{
    CanMessage::CanMessage(uint32_t id_in, std::string name_in, std::vector<can_signal> can_signals_in)
    {
        id = id_in;
        name = name_in;
        can_signals = can_signals_in;
    }

    std::optional<CanMessage> decode_can_message(CAN_frame_t rx_frame, std::vector<CanMessage> can_db_messages)
    {
        std::vector<can_signal> can_signals;
        for (uint16_t j = 0; j < can_db_messages.size(); j++)
        {
            if (rx_frame.MsgID == can_db_messages[j].id)
            {
                uint64_t can_data = join_together_CAN_data_bytes_from_array(rx_frame);
                for (uint16_t i = 0; i < can_db_messages[j].can_signals.size(); i++)
                {
                    robast_can_msgs::can_signal can_signal;
                    can_signal.bit_start = can_db_messages[j].can_signals[i].bit_start;
                    can_signal.bit_length = can_db_messages[j].can_signals[i].bit_length;
                    can_signal.name = can_db_messages[j].can_signals[i].name;
                    can_signal.data = (can_data << can_signal.bit_start) >> (64 - can_signal.bit_length);
                    can_signals.push_back(can_signal);
                }
                return CanMessage(rx_frame.MsgID, can_db_messages[j].name, can_signals);
            }
        }
        return std::nullopt;
    }

    std::optional<CAN_frame_t> encode_can_message(CanMessage can_message, std::vector<CanMessage> can_db_messages)
    {
        CAN_frame_t can_frame;

        uint64_t can_data = join_together_CAN_data_from_CAN_message(can_message);
        uint8_t *can_data_bytes = (uint8_t *)&can_data;

        uint8_t dlc = get_dlc_of_can_message(can_message);

        for (uint16_t j = 0; j < can_db_messages.size(); j++)
        {
            if (can_message.id == can_db_messages[j].id)
            {
                can_frame.MsgID = can_message.id;
                can_frame.FIR.B.DLC = dlc;
                can_frame.FIR.B.FF = CAN_frame_format_t::CAN_frame_std;
                for (uint16_t i = 0; i < dlc; i++)
                {
                    can_frame.data.u8[i] = can_data_bytes[dlc-i];
                }
                return can_frame;
            }
        }
        return std::nullopt;
    }

    uint64_t join_together_CAN_data_bytes_from_array(CAN_frame_t rx_frame)
    {
        uint64_t can_data = 0;
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++)
        {
            uint64_t can_byte = rx_frame.data.u8[i];
            can_data = (can_byte << (7-i)*8) | can_data;            
        }
        return can_data;
    }

    uint64_t join_together_CAN_data_from_CAN_message(CanMessage can_message)
    {  
        uint64_t can_data = 0;
        for (uint8_t i = 0; i < can_message.can_signals.size(); i++)
        {
            uint64_t can_signal_data = can_message.can_signals[i].data;
            can_data = can_data | (can_signal_data << (64 - can_message.can_signals[i].bit_start - can_message.can_signals[i].bit_length));
        }
        return can_data;
    }

    uint8_t get_dlc_of_can_message(CanMessage can_message)
    {
        float num_of_bytes_can_message = can_message.can_signals.back().bit_start + can_message.can_signals.back().bit_length;
        return std::ceil(num_of_bytes_can_message / 8);
    }
}