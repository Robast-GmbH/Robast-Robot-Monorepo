#include "../include/can/can_signal.hpp"

namespace robast_can_msgs
{
  void CanSignal::set_data(uint64_t data)
  {
    this->_data = data;
  }

  uint64_t CanSignal::get_data() const
  {
    return this->_data;
  }

  uint8_t CanSignal::get_bit_start() const
  {
    return this->_bit_start;
  }

  uint8_t CanSignal::get_bit_length() const
  {
    return this->_bit_length;
  }
}   // namespace robast_can_msgs
