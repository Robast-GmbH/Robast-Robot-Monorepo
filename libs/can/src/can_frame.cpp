#include "../include/can/can_frame.hpp"

namespace robast_can_msgs
{
  CanFrame::CanFrame(uint32_t id_in, uint8_t dlc_in, uint8_t data_in[])
  {
    this->id_ = id_in;
    this->dlc_ = dlc_in;
    this->data_ = (uint8_t*) malloc(CAN_STD_MSG_DLC_MAXIMUM * sizeof(uint8_t));
    std::memcpy(this->data_, data_in, CAN_STD_MSG_DLC_MAXIMUM);
  }

  CanFrame::~CanFrame()
  {
    free(this->data_);
  }

  void CanFrame::set_data(uint8_t* data)
  {
    this->data_ = data;
  }

  uint8_t* CanFrame::get_data() const
  {
    return this->data_;
  }

  uint32_t CanFrame::get_id() const
  {
    return this->id_;
  }

  uint8_t CanFrame::get_dlc() const
  {
    return this->dlc_;
  }
}   // namespace robast_can_msgs
