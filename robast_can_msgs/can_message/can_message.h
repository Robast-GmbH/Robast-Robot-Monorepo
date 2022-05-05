#ifndef CAN_MESSAGE_HPP_
#define CAN_MESSAGE_HPP_

#include <optional>
#include <string>
#include <vector>

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
            CanMessage(uint32_t id, std::string name, std::vector<can_signal> can_signals);

            uint32_t id;
            std::string name;
            std::vector<can_signal> can_signals;

            uint32_t get_id();
    };

    /**
     * @brief Decodes CAN message from a 8 Byte Bitstream
     *
     * @param rx_frame The received CAN data frame
     * @param can_db CAN database to decode message with
     * @param name name of the CAN message
     * @return std::optional<CanMessage>
     */
    std::optional<CanMessage> decode_can_message(CAN_frame_t rx_frame, std::vector<CanMessage> can_db);

    /**
     * @brief Joins together the data bytes from the CAN bus
     *
     * @param rx_frame The received CAN data frame
     * @return 8Byte CAN data joined together 
     */
    uint64_t join_together_CAN_data_bytes(CAN_frame_t rx_frame);
}

#endif /* CAN_MESSAGE_HPP_ */