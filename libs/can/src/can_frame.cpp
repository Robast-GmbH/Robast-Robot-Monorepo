#include "../include/can_frame.h"

namespace robast_can_msgs
{
    CanFrame::CanFrame(uint32_t id_in, uint8_t dlc_in, uint8_t data_in[])
    {
        this->id_ = id_in;
        this->dlc_ = dlc_in;
        this->data_ = (uint8_t*) malloc (8 * sizeof(uint8_t));
        std::memcpy(this->data_, data_in, 8);
    }

    CanFrame::~CanFrame()
    {
        free(this->data_);
    }

    void CanFrame::set_data(uint8_t* data)
    {
        this->data_ = data;
    }
	
	uint8_t* CanFrame::get_data()
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
}