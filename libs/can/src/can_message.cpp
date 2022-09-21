#include "../include/can_message.h"

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
}