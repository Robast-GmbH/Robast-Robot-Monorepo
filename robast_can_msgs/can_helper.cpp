#include "can_helper.h"

namespace robast_can_msgs
{
    std::optional<CanMessage> decode_can_message(uint32_t msg_id, uint8_t data[], uint8_t dlc, std::vector<CanMessage> can_db_messages)
    {
        std::vector<CanSignal> can_signals;
        for (uint16_t j = 0; j < can_db_messages.size(); j++)
        {
            if (msg_id == can_db_messages[j].id)
            {
                uint64_t can_msg_data = join_together_CAN_data_bytes_from_array(data, dlc);
                for (uint16_t i = 0; i < can_db_messages[j].can_signals.size(); i++)
                {
                    uint8_t bit_start = can_db_messages[j].can_signals[i].bit_start;
                    uint8_t bit_length = can_db_messages[j].can_signals[i].bit_length;
                    uint64_t can_signal_data = (can_msg_data << bit_start) >> (64 - bit_length);

                    robast_can_msgs::CanSignal can_signal = CanSignal(bit_start, bit_length, can_signal_data);
                    can_signals.push_back(can_signal);
                }
                return CanMessage(msg_id, dlc, can_signals);
            }
        }
        return std::nullopt;
    }

    std::optional<CanFrame> encode_can_message_into_can_frame(CanMessage can_message, std::vector<CanMessage> can_db_messages)
    { 
        for (uint16_t j = 0; j < can_db_messages.size(); j++)
        {
            if (can_message.id == can_db_messages[j].id)
            {
                uint64_t can_data = join_together_CAN_data_from_CAN_message(can_message);
                uint8_t* can_data_bytes = (uint8_t*) malloc (8 * sizeof(uint8_t));
                u64_to_eight_bytes(can_data, can_data_bytes);
                return CanFrame(can_message.id, can_message.dlc, can_data_bytes);
            }
        }
        return std::nullopt;
    }

    /* The USB-CAN Controller is controlled via simple ASCII commands over the serial port.
    * The full command list can be found here: https://www.fischl.de/usbtin/
    * The command for transmitting standard (11 bit) frame should look like:
    *   tiiildd..[CR]
    *       iii: Identifier in hexadecimal format (000-7FF)
    *       l: Data length (0-8)
    *       dd: Data byte value in hexadecimal format (00-FF)
    */
    std::optional<std::string> encode_can_message_into_ascii_command(CanMessage can_message, std::vector<CanMessage> can_db_messages)
    {
        for (uint16_t j = 0; j < can_db_messages.size(); j++)
        {
            if (can_message.id == can_db_messages[j].id)
            {
                uint64_t can_data = join_together_CAN_data_from_CAN_message(can_message);
                can_data = can_data >> ((8 - can_message.dlc) * 8);

                std::string ascii_command = "t";
                
                ascii_command.append(uint_to_hex_string(can_message.id, 3));
                ascii_command.append(std::to_string(can_message.dlc));
                ascii_command.append(uint_to_hex_string(can_data, can_message.dlc*2));
                ascii_command.append("\r");

                return ascii_command;
            }
        }
        return std::nullopt;
    }

    /* The USB-CAN Controller is controlled via simple ASCII commands over the serial port.
    * The full command list can be found here: https://www.fischl.de/usbtin/
    * The command for transmitting standard (11 bit) frame should look like:
    *   tiiildd..[CR]
    *       iii: Identifier in hexadecimal format (000-7FF)
    *       l: Data length (0-8)
    *       dd: Data byte value in hexadecimal format (00-FF)
    */
    std::optional<CanMessage> decode_ascii_command_into_can_message(const char* ascii_command, uint8_t ascii_command_length, std::vector<CanMessage> can_db_messages)
    {
        if (ascii_command_length > 5 || ascii_command[0] != 't')
        {
            std::string id_as_hex_string = std::string(ascii_command + 1, 3);
            uint16_t can_msg_id = hex_string_to_unsigned_int<uint16_t>(id_as_hex_string);
            for (uint16_t j = 0; j < can_db_messages.size(); j++)
            {
                if (can_msg_id == can_db_messages[j].id)
                {
                    std::string dlc_as_hex_string = std::string(ascii_command + 4, 1);
                    uint8_t dlc = hex_string_to_unsigned_int<uint8_t>(dlc_as_hex_string);

                    std::string data_as_hex_string = std::string(ascii_command + 5, dlc);
                    uint64_t data = hex_string_to_unsigned_int<uint64_t>(data_as_hex_string);
                }
            }
        }
        else
        {
            return std::nullopt;
        }
    }

    template <typename T>
    T hex_string_to_unsigned_int(std::string hex_string)
    {
        T result;
        result = std::stoul(hex_string, nullptr, 16);
        return result;
    }

    // uint16_t hex_string_to_uint16_t(std::string hex_string)
    // {
    //     uint16_t result;
    //     result = std::stoul(hex_string, nullptr, 16);
    //     return result;
    // }

    // uint64_t hex_string_to_uint64_t(std::string hex_string)
    // {
    //     uint64_t result;
    //     result = std::stoul(hex_string, nullptr, 16);
    //     return result;
    // }

    std::string uint_to_hex_string(uint32_t input, int num_of_digits)
    {
        std::stringstream stream;
        stream << std::setfill('0') << std::setw(num_of_digits) << std::uppercase << std::hex << input;
        return stream.str();
    }

    std::string uint_to_hex_string(uint64_t input, int num_of_digits)
    {
        std::stringstream stream;
        stream << std::setfill('0') << std::setw(num_of_digits) << std::uppercase << std::hex << input;
        return stream.str();
    }

    void u64_to_eight_bytes(uint64_t input, uint8_t *result)
    {
        uint64_t big_endian;
        int check_for_little_endian = 1;
        // little endian if true
        if (*(char *)&check_for_little_endian == 1)
        {
            SwapEndian<uint64_t>(input);
            big_endian = input; 
        }
        else
        {
            big_endian = input;
        }

        std::memcpy(result, &big_endian, sizeof(input) );    
    }

    template <typename T>
    void SwapEndian(T &val) {
        union U {
            T val;
            std::array<uint8_t, sizeof(T)> raw;
        } src, dst;

        src.val = val;
        std::reverse_copy(src.raw.begin(), src.raw.end(), dst.raw.begin());
        val = dst.val;
    }

    uint64_t join_together_CAN_data_bytes_from_array(uint8_t data[], uint8_t dlc)
    {
        uint64_t can_data = 0;
        for (int i = 0; i < dlc; i++)
        {
            uint64_t can_byte = data[i];
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
}