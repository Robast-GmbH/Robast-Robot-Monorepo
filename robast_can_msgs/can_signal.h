#ifndef CAN_SIGNAL_HPP_
#define CAN_SIGNAL_HPP_

#include <cmath>

namespace robast_can_msgs
{
    class CanSignal
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanSignal class
             */
            CanSignal(uint8_t bit_start_in, uint8_t bit_length_in, uint64_t data_in) : bit_start{bit_start_in}, bit_length{bit_length_in}, data{data_in} {}

            const uint8_t bit_start;
            const uint8_t bit_length; // number of bits for this can_signal
            uint64_t data;
    };
}

#endif /* CAN_SIGNAL_HPP_ */