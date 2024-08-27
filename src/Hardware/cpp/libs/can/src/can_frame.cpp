#include "../include/can/can_frame.hpp"

namespace robast_can_msgs
{
  CanFrame::CanFrame(uint32_t id, uint8_t dlc, uint8_t data[])
  {
    this->_id = id;
    this->_dlc = dlc;
    this->_data = (uint8_t*) malloc(CAN_STD_MSG_DLC_MAXIMUM * sizeof(uint8_t));
    std::memcpy(this->_data, data, CAN_STD_MSG_DLC_MAXIMUM);
  }

  CanFrame::~CanFrame()
  {
    free(this->_data);
  }

  void CanFrame::set_data(uint8_t* data)
  {
    this->_data = data;
  }

  uint8_t* CanFrame::get_data() const
  {
    return this->_data;
  }

  uint32_t CanFrame::get_id() const
  {
    return this->_id;
  }

  uint8_t CanFrame::get_dlc() const
  {
    return this->_dlc;
  }
}   // namespace robast_can_msgs
