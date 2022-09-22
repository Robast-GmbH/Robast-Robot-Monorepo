#ifndef CAN_FRAME_HPP_
#define CAN_FRAME_HPP_

#include <cmath>
#include <cstdint>
#include <cstring>

namespace robast_can_msgs
{
    class CanFrame
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanFrame class
             */
            CanFrame(uint32_t id_in, uint8_t dlc_in, uint8_t data_in[]);
            /**
             * @brief A destructor for robast_can_msgs::CanFrame class
             */
            ~CanFrame();

            /**
             * @brief A setter function for the data of the CanFrame
             */
            void set_data(uint8_t* data);

            /**
             * @brief A getter function for the data of the CanFrame
             */
			uint8_t* get_data();

            /**
             * @brief A getter function for the id of the CanFrame
             */
			uint32_t get_id() const;

            /**
             * @brief A getter function for the dlc of the CanFrame
             */
			uint8_t get_dlc() const;
            
        protected:
            uint8_t* data_;
            uint8_t dlc_;
            uint32_t id_;
    };

}

#endif /* CAN_FRAME_HPP_ */