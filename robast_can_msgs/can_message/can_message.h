#ifndef CAN_MESSAGE_HPP_
#define CAN_MESSAGE_HPP_

#include <optional>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <array>
#include <algorithm>

#include "robast_can_msgs/can_signal/can_signal.h"
#include "robast_can_msgs/can_frame/can_frame.h"

namespace robast_can_msgs
{
    class CanMessage
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanMessage class
             */
            CanMessage(uint32_t id_in, uint8_t dlc_in, std::vector<CanSignal> can_signals_in) : id{id_in}, dlc{dlc_in}, can_signals{can_signals_in} {}

            const uint32_t id;
            const uint8_t dlc;
            std::vector<CanSignal> can_signals;
    };

    /**
     * @brief Decodes CAN message from a 8 Byte Bitstream
     *
     * @param msg_id The ID of the received can msg
     * @param data The array of data bytes of the received msg
     * @param dlc The number of data bytes of the received msg
     * @param can_db_messages CAN messages from CAN database to decode the message with
     * @return std::optional<CanMessage>
     */
    std::optional<CanMessage> decode_can_message(uint32_t msg_id, uint8_t data[], uint8_t dlc, std::vector<CanMessage> can_db_messages);

    /**
     * @brief Encodes CAN message into CAN_frame_t with 8 Byte Bitstream
     *
     * @param can_message The can_message to be encoded
     * @param can_db_messages CAN messages from CAN database to encode the message with
     * @return std::optional<CanFrame>
     */
    std::optional<CanFrame> encode_can_message(CanMessage can_message, std::vector<CanMessage> can_db_messages);    

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
     * @brief Swap Endian
     *
     * @param val The value that's supposed to be swapped
     */
    template <typename T>
    void SwapEndian(T &val);
}

#endif /* CAN_MESSAGE_HPP_ */