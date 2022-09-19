#include "can_frame.h"

namespace robast_can_msgs
{
    void CanFrame::set_data(uint8_t* data)
    {
        this->data_ = data;
    }
	
	uint8_t* CanFrame::get_data()
    {
        return this->data_;
    }
}