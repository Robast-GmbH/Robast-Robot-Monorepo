#ifndef CAN__CAN_FRAME_HPP_
#define CAN__CAN_FRAME_HPP_

#include <cmath>
#include <cstdint>
#include <cstring>

#include "can/can_db_defines.hpp"

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
    uint8_t* get_data() const;

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

}   // namespace robast_can_msgs

#endif   // CAN__CAN_FRAME_HPP_
