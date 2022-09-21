#ifndef CAN_SIGNAL_HPP_
#define CAN_SIGNAL_HPP_

#include <cmath>
#include <cstdint>

namespace robast_can_msgs
{
    class CanSignal
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanSignal class
             */
            CanSignal(uint8_t bit_start_in, uint8_t bit_length_in, uint64_t data_in) : bit_start_{bit_start_in}, bit_length_{bit_length_in}, data_{data_in} {}

            /**
             * @brief A destructor for robast_can_msgs::CanSignal class
             */
            ~CanSignal() = default;

            /**
             * @brief A setter function for the data of the CanSignal
             */
            void set_data(uint64_t data);

            /**
             * @brief A getter function for the data of the CanSignal
             */
			uint64_t get_data();

            /**
             * @brief A getter function for the bit_start of the CanSignal
             */
			uint8_t get_bit_start();

            /**
             * @brief A getter function for the bit_length of the CanSignal
             */
			uint8_t get_bit_length();

        protected:
            uint64_t data_;
            uint8_t bit_start_;
            uint8_t bit_length_; // number of bits for this can_signal
    };
}

#endif /* CAN_SIGNAL_HPP_ */