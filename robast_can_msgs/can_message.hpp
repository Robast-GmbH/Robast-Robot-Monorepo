#ifndef CAN_MESSAGE_HPP_
#define CAN_MESSAGE_HPP_

#include <string>
#include <vector>

namespace robast_can_msgs
{
    struct can_signal
        {
            std::string name;
            uint8_t bit_start_LSB;
            uint8_t bit_length; // number of bits for this can_signal
            uint64_t data;
        };

    class CanMessage
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanMessage class
             */
            CanMessage(uint16_t id, std::string name, std::vector<can_signal> can_signals);

            uint16_t id;
            std::string name;
            std::vector<can_signal> can_signals;            
    };

    /**
     * @brief Decodes can message from a 8 Byte Bitstream
     *
     * @param id ID of the can message
     * @param name name of the can message
     * @param can_data 8 Byte Bitstream where to decode data from
     * @return CanMessage
     */
    CanMessage decode_can_message(uint16_t id, std::string name, uint64_t can_data);
}

#endif /* CAN_MESSAGE_HPP_ */