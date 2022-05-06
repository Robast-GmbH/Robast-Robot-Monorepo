#ifndef CAN_MESSAGE_HPP_
#define CAN_MESSAGE_HPP_

#include <optional>
#include <string>
#include <vector>
#include <cmath>

#ifdef IS_TEST
#include "robast_can_msgs/can_mock/can_mock.h"
using namespace can_mock;
#else
#include <CAN.h>
#endif

namespace robast_can_msgs
{
    struct can_signal
        {
            std::string name;
            uint8_t bit_start;
            uint8_t bit_length; // number of bits for this can_signal
            uint64_t data;
        };

    class CanMessage
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanMessage class
             */
            CanMessage(uint32_t id_in, std::string name_in, std::vector<can_signal> can_signals_in);

            uint32_t id;
            std::string name;
            std::vector<can_signal> can_signals;
    };

    /**
     * @brief Decodes CAN message from a 8 Byte Bitstream
     *
     * @param rx_frame The received CAN data frame
     * @param can_db_messages CAN messages from CAN database to decode the message with
     * @return std::optional<CanMessage>
     */
    std::optional<CanMessage> decode_can_message(CAN_frame_t rx_frame, std::vector<CanMessage> can_db_messages);

    /**
     * @brief Encodes CAN message into CAN_frame_t with 8 Byte Bitstream
     *
     * @param can_message The can_message to be encoded
     * @param can_db_messages CAN messages from CAN database to encode the message with
     * @return std::optional<CAN_frame_t>
     */
    std::optional<CAN_frame_t> encode_can_message(CanMessage can_message, std::vector<CanMessage> can_db_messages);    

    /**
     * @brief Joins together the data bytes from the CAN bus
     *
     * @param rx_frame The received CAN data frame
     * @return 8Byte CAN data joined together 
     */
    uint64_t join_together_CAN_data_bytes_from_array(CAN_frame_t rx_frame);

    /**
     * @brief Joins together the CAN data that is encapsulated in the CanMessage class
     *
     * @param can_message The CAN message encapsulated into the CanMessage class
     * @return All CAN data joined into 64 bit stream 
     */
    uint64_t join_together_CAN_data_from_CAN_message(CanMessage can_message);

    /**
     * @brief Gets DLC for CanMessage
     *
     * @param can_message The CAN message encapsulated into the CanMessage class
     * @return DLC for the CAN message
     */
    uint8_t get_dlc_of_can_message(CanMessage can_message);
}

#endif /* CAN_MESSAGE_HPP_ */