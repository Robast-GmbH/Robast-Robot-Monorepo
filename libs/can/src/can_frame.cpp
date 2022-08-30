#include "include/can_frame.hpp"

namespace robast_can_msgs
{
    void CanFrame::set_data(uint8_t* data)
    {
        this->data_ = data;
    }
}