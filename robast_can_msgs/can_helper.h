#ifndef CAN_HELPER_HPP_
#define CAN_HELPER_HPP_

#include <optional>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <array>
#include <algorithm>
#include <iomanip>
#include <iostream> //only for debugging

#include "can_message.h"
#include "can_frame.h"

namespace robast_can_msgs
{
    
    enum can_baudrate_usb_to_can_interface 
    {
        can_baud_10kbps = 0,
        can_baud_20kbps = 1,
        can_baud_50kbps = 2,
        can_baud_100kbps = 3,
        can_baud_125kbps = 4,
        can_baud_250kbps = 5,
        can_baud_500kbps = 6,
        can_baud_800kbps = 7,
        can_baud_1000kbps = 8
    };

    /**
     * @brief Decodes CAN message from a 8 Byte bitstream
     *
     * @param msg_id The ID of the received can msg
     * @param data The array of data bytes of the received msg
     * @param dlc The number of data bytes of the received msg
     * @param can_db_messages CAN messages from CAN database to decode the message with
     * @return std::optional<CanMessage>
     */
    std::optional<CanMessage> decode_can_message(uint32_t msg_id, uint8_t data[], uint8_t dlc, std::vector<CanMessage> can_db_messages);

    /**
     * @brief Encodes CAN message into CAN_frame_t with 8 Byte bitstream
     *
     * @param can_message The can_message to be encoded
     * @param can_db_messages CAN messages from CAN database to encode the message with
     * @return std::optional<CanFrame>. Only contains a value if the can_message id exists in the can_db.
     */
    std::optional<CanFrame> encode_can_message_into_can_frame(CanMessage can_message, std::vector<CanMessage> can_db_messages);  

    /**
     * @brief Encodes CAN message into a ASCII command, which is needed for the USB-CAN adapter
     *
     * @param can_message The can_message to be encoded
     * @param can_db_messages CAN messages from CAN database to encode the message with
     * @return std::optional<std::string>. Only contains a value if the can_message id exists in the can_db.
     */
    std::optional<std::string> encode_can_message_into_ascii_command(CanMessage can_message, std::vector<CanMessage> can_db_messages);

    /**
     * @brief Decodes an ASCII command sent from the USB-CAN adapter into a CAN message
     *
     * @param ascii_commands_as_string The ASCII command to be encoded
     * @param ascii_commands_length The number of chars the ASCII command contains
     * @param can_db_messages CAN messages from CAN database to encode the message with
     * @return std::optional<CanMessage>. Only contains a value if the can_message id exists in the can_db and the ascii command has the proper format.
     */
    std::optional<CanMessage> decode_single_ascii_command_into_can_message(std::string ascii_commands_as_string, uint8_t ascii_commands_length, std::vector<CanMessage> can_db_messages); 

    /**
     * @brief Decodes string that may contain more than one ASCII commant sent from the USB-CAN adapter into a CAN message
     *
     * @param ascii_command The string with ASCII commands to be encoded
     * @param can_msgs_id The CAN message ID of the expected can message
     * @param dlc The dlc of the expected can message
     * @param can_db_messages CAN messages from CAN database to encode the message with
     * @return std::optional<CanMessage>. Only contains a value if the can_message id exists in the can_db and the ascii command has the proper format.
     */
    std::vector<CanMessage> decode_multiple_ascii_commands_into_can_messages(std::string ascii_commands, uint32_t can_msgs_id, uint8_t dlc, std::vector<CanMessage> can_db_messages);

    /**
     * @brief Joins together the data bytes from the CAN bus
     *
     * @param data The received CAN data
     * @param dlc The number of data bytes
     * @return 8Byte CAN data joined together 
     */
    uint64_t join_together_CAN_data_bytes_from_array(uint8_t data[], uint8_t dlc);

    /**
     * @brief Joins together the CAN data that is encapsulated in the CanMessage class
     *
     * @param can_message The CAN message encapsulated into the CanMessage class
     * @return All CAN data joined into 64 bit stream 
     */
    uint64_t join_together_CAN_data_from_CAN_message(CanMessage can_message);

    /**
     * @brief Converts a uint64 into 8 byte array
     *
     * @param input The uint64 to be converted
     * @param result Pointer to the 8 byte array for the result
     */
    void u64_to_eight_bytes(uint64_t input, uint8_t *result);

    /**
     * @brief Convert hex data contained in a string to an unsigned integer
     *
     * @param hex_string The input string that contains the data in hex format.
     * @return The data, that was contained in the string, as an unsigned integer.
     */
    template <typename T>
    T hex_string_to_unsigned_int(std::string hex_string);

    /**
     * @brief Swap Endian
     *
     * @param val The value that's supposed to be swapped
     */
    template <typename T>
    void SwapEndian(T &val);

    /**
     * @brief Converts a uint32 into a string where the numbers are represented in hex format in uppercase
     *
     * @param input The uint64 to be converted
     * @param num_of_digits The number of digits the converted hex number should have. This enables leading zeros.
     */
    std::string uint_to_hex_string(uint32_t input, int num_of_digits);

    /**
     * @brief Converts a uint64 into a string where the numbers are represented in hex format in uppercase
     *
     * @param input The uint64 to be converted
     * @param num_of_digits The number of digits the converted hex number should have. This enables leading zeros.
     */
    std::string uint_to_hex_string(uint64_t input, int num_of_digits);
    
    /**
     * @brief Assigns the data that is contained in the uint64_t parameter to the can signals
     *
     * @param can_msg_data The uint64_t parameter that contains the data for the can signals
     * @param can_db_messages  The vector containing all can_messages that are defined in the CAN database
     * @param can_msgs_index The index of the CAN message we want to decode
     * @return A Vector of all CanSignals filled with the data that is contained in the can_msgs_data parameter
     */
    std::vector<CanSignal> assign_data_to_can_signals(uint64_t can_msg_data, std::vector<CanMessage> can_db_messages, uint16_t can_msgs_index);

}

#endif /* CAN_HELPER_HPP_ */