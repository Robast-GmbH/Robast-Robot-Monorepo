#include "../include/can_signal.h"

namespace robast_can_msgs
{
    void CanSignal::set_data(uint64_t data)
    {
        this->data_ = data;
    }
	
	uint64_t CanSignal::get_data()
    {
        return this->data_;
    }

    uint8_t CanSignal::get_bit_start()
    {
        return this->bit_start_;
    }

    uint8_t CanSignal::get_bit_length()
    {
        return this->bit_length_;
    }
}