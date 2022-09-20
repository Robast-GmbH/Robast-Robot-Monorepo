#ifndef CAN_FRAME_HPP_
#define CAN_FRAME_HPP_

#include <cmath>
#include <cstdint>

namespace robast_can_msgs
{
    class CanFrame
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanFrame class
             */
            CanFrame(uint32_t id_in, uint8_t dlc_in, uint8_t data_in[]) : id{id_in}, dlc{dlc_in}, data_{data_in} {}

            const uint32_t id;
            const uint8_t dlc;

            /**
             * @brief A setter function for the data of the CanFrame
             */
            void set_data(uint8_t* data);

            /**
             * @brief A getter function for the data of the CanFrame
             */
			uint8_t* get_data();
            
        protected:
            uint8_t* data_;
    };

}

#endif /* CAN_FRAME_HPP_ */