#include "../include/can_helper.h"

namespace robast_can_msgs
{
    std::optional<CanMessage> decode_can_message(uint32_t msg_id, uint8_t data[], uint8_t dlc, std::vector<CanMessage> can_db_messages)
    {
        for (uint16_t can_msgs_index = 0; can_msgs_index < can_db_messages.size(); can_msgs_index++)
        {
            if (msg_id == can_db_messages[can_msgs_index].get_id())
            {
                uint64_t can_msg_data = join_together_CAN_data_bytes_from_array(data, dlc);
                std::vector<CanSignal> can_signals = assign_data_to_can_signals(can_msg_data, can_db_messages, can_msgs_index);
                return CanMessage(msg_id, dlc, can_signals);
            }
        }
        return std::nullopt;
    }

    std::vector<CanSignal> assign_data_to_can_signals(uint64_t can_msg_data, std::vector<CanMessage> can_db_messages, uint16_t can_msgs_index)
    {
        std::vector<CanSignal> can_signals;
        uint8_t dlc = can_db_messages[can_msgs_index].get_dlc();
        for (uint16_t i = 0; i < can_db_messages[can_msgs_index].get_can_signals().size(); i++)
        {
            uint8_t bit_start = can_db_messages[can_msgs_index].get_can_signals()[i].get_bit_start();
            uint8_t bit_length = can_db_messages[can_msgs_index].get_can_signals()[i].get_bit_length();
            
            uint64_t can_signal_data = (can_msg_data << (bit_start + 8*(8-dlc))) >> (64 - bit_length);

            robast_can_msgs::CanSignal can_signal = CanSignal(bit_start, bit_length, can_signal_data);
            can_signals.push_back(can_signal);
        }
        return can_signals;
    }

    std::optional<CanFrame> encode_can_message_into_can_frame(CanMessage can_message, std::vector<CanMessage> can_db_messages)
    { 
        for (uint16_t j = 0; j < can_db_messages.size(); j++)
        {
            if (can_message.get_id() == can_db_messages[j].get_id())
            {
                uint64_t can_data = join_together_CAN_data_from_CAN_message(can_message);
                uint8_t* can_data_bytes = (uint8_t*) malloc (8 * sizeof(uint8_t));
                u64_to_eight_bytes(can_data, can_data_bytes);
                return CanFrame(can_message.get_id(), can_message.get_dlc(), can_data_bytes);
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
            if (can_message.get_id() == can_db_messages[j].get_id())
            {
                uint64_t can_data = join_together_CAN_data_from_CAN_message(can_message);
                can_data = can_data >> ((8 - can_message.get_dlc()) * 8);

                std::string ascii_command = "t";
                
                ascii_command.append(uint_to_hex_string(can_message.get_id(), 3));
                ascii_command.append(std::to_string(can_message.get_dlc()));
                ascii_command.append(uint_to_hex_string(can_data, can_message.get_dlc()*2));

                return ascii_command;
            }
        }
        return std::nullopt;
    }

    /*
    * The USB-CAN Controller is controlled via simple ASCII commands over the serial port.
    * The full command list can be found here: https://www.fischl.de/usbtin/
    * The command for transmitting standard (11 bit) frame should look like:
    *   tiiildd..[CR]
    *       iii: Identifier in hexadecimal format (000-7FF)
    *       l: Data length (0-8)
    *       dd: Data byte value in hexadecimal format (00-FF)
    */
    std::optional<CanMessage> decode_single_ascii_command_into_can_message(std::string ascii_commands_as_string, uint8_t ascii_commands_length, std::vector<CanMessage> can_db_messages)
    {
        // I don't know why, but:
        // Some of the serial read results have 2 beginning bytes that contain the ASCII code 7
        // Therefore remove the first to bytes from the read serial command
        if ((ascii_commands_as_string[0] == 7) && (ascii_commands_as_string[1] == 7) && (ascii_commands_as_string.length() > 2))
        {
            ascii_commands_as_string.erase(ascii_commands_as_string.begin(), ascii_commands_as_string.begin() + 2);
        }

        const char* ascii_commands = &ascii_commands_as_string[0];
        if (ascii_commands_length > 5 && ascii_commands[0] == 't')
        {
            std::string id_as_hex_string = std::string(ascii_commands + 1, 3);
            uint32_t can_msg_id = hex_string_to_unsigned_int<uint32_t>(id_as_hex_string);
            for (uint16_t can_msgs_index = 0; can_msgs_index < can_db_messages.size(); can_msgs_index++)
            {
                if (can_msg_id == can_db_messages[can_msgs_index].get_id())
                {
                    std::string dlc_as_hex_string = std::string(ascii_commands + 4, 1);
                    uint8_t dlc = hex_string_to_unsigned_int<uint8_t>(dlc_as_hex_string);

                    std::string data_as_hex_string = std::string(ascii_commands + 5, dlc*2);
                    uint64_t can_msg_data = hex_string_to_unsigned_int<uint64_t>(data_as_hex_string);

                    std::vector<CanSignal> can_signals = assign_data_to_can_signals(can_msg_data, can_db_messages, can_msgs_index);

                    return CanMessage(can_msg_id, dlc, can_signals);
                }
            }
        }
        return std::nullopt;
    }

    std::vector<CanMessage> decode_multiple_ascii_commands_into_can_messages(std::string ascii_commands, uint32_t can_msgs_id, uint8_t dlc, std::vector<CanMessage> can_db_messages)
    {
        std::vector<CanMessage> received_can_msgs;
        /*
        * The USB-CAN Controller is controlled via simple ASCII commands over the serial port.
        * The full command list can be found here: https://www.fischl.de/usbtin/
        * The command for transmitting standard (11 bit) frame should look like:
        *   tiiildd..[CR]
        *       iii: Identifier in hexadecimal format (000-7FF)
        *       l: Data length (0-8)
        *       dd: Data byte value in hexadecimal format (00-FF)
        *
        * Therefore a received ASCII command of a CAN message has 2*DLC ("ddd...") + 6 ("t"+"iii"+"l"+"[CR]") bytes
        */
        uint8_t expected_num_of_bytes_ascii_cmd = 2 * dlc + 6;

        // If we received more than one CAN message, we need to split the received serial_read_ascii_command
        if (ascii_commands.length() > expected_num_of_bytes_ascii_cmd)
        {
            uint8_t num_of_received_can_msgs = ascii_commands.length() / expected_num_of_bytes_ascii_cmd;
            
            for (uint8_t i = 0; i < num_of_received_can_msgs; i++)
            {
                std::optional<CanMessage> decoded_can_message = decode_single_ascii_command_into_can_message(ascii_commands, ascii_commands.length(), can_db_messages);
                if (decoded_can_message.has_value())
                {
                    if (decoded_can_message.value().get_id() == can_msgs_id)
                    {
                        received_can_msgs.push_back(decoded_can_message.value());
                    }
                }
                ascii_commands.erase(ascii_commands.begin(), ascii_commands.begin() + expected_num_of_bytes_ascii_cmd);
            }
        }
        else
        {
            std::optional<CanMessage> decoded_can_message = decode_single_ascii_command_into_can_message(ascii_commands, ascii_commands.length(), can_db_messages);
            if (decoded_can_message.has_value())
            {
                if (decoded_can_message.value().get_id() == can_msgs_id)
                {
                    received_can_msgs.push_back(decoded_can_message.value());
                }
            }
        }
        return received_can_msgs;
    }

    template <typename T>
    T hex_string_to_unsigned_int(std::string hex_string)
    {
        T result;
        result = std::stoul(hex_string, nullptr, 16);
        return result;
    }

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

        std::cout << "vor memcopy result[0]: " << (int)result[0] << std::endl;
        std::memcpy(result, &big_endian, sizeof(big_endian));
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
        for (uint8_t i = 0; i < can_message.get_can_signals().size(); i++)
        {
            uint64_t can_signal_data = can_message.get_can_signals()[i].get_data();
            can_data = can_data | (can_signal_data << (64 - can_message.get_can_signals()[i].get_bit_start() - can_message.get_can_signals()[i].get_bit_length()));
        }
        return can_data;
    }
}