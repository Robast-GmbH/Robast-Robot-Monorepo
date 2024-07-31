#include "../include/can/can_signal.hpp"

namespace robast_can_msgs
{
  void CanSignal::set_data(uint64_t data)
  {
    this->data_ = data;
  }

  uint64_t CanSignal::get_data() const
  {
    return this->data_;
  }

  uint8_t CanSignal::get_bit_start() const
  {
    return this->bit_start_;
  }

  uint8_t CanSignal::get_bit_length() const
  {
    return this->bit_length_;
  }
}   // namespace robast_can_msgs
