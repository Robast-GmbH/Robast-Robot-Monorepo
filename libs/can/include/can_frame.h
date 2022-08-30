#ifndef CAN_FRAME_HPP_
#define CAN_FRAME_HPP_

#include <cmath>

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
            void set_data(uint8_t data);
            
        protected:
            uint8_t* data_;
    };

}

#endif /* CAN_FRAME_HPP_ */