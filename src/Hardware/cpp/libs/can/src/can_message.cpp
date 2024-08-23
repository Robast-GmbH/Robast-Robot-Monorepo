#include "../include/can/can_message.hpp"

namespace robast_can_msgs
{
  void CanMessage::set_can_signals(const std::vector<CanSignal> &can_signals)
  {
    this->_can_signals = can_signals;
  }

  std::vector<CanSignal> CanMessage::get_can_signals() const
  {
    return this->_can_signals;
  }

  uint32_t CanMessage::get_id() const
  {
    return this->_id;
  }

  uint8_t CanMessage::get_dlc() const
  {
    return this->_dlc;
  }
}   // namespace robast_can_msgs
