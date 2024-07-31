#include "../include/can/can_message.hpp"

namespace robast_can_msgs
{
  void CanMessage::set_can_signals(const std::vector<CanSignal> &can_signals)
  {
    this->can_signals_ = can_signals;
  }

  std::vector<CanSignal> CanMessage::get_can_signals() const
  {
    return this->can_signals_;
  }

  uint32_t CanMessage::get_id() const
  {
    return this->id_;
  }

  uint8_t CanMessage::get_dlc() const
  {
    return this->dlc_;
  }
}   // namespace robast_can_msgs
