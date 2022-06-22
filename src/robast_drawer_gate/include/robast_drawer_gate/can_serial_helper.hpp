#ifndef ROBAST_CAN_SERIAL_HELPER__CAN_SERIAL_HELPER_HPP_
#define ROBAST_CAN_SERIAL_HELPER__CAN_SERIAL_HELPER_HPP_

namespace robast_drawer_gate
{
    enum can_baudrate_usb_to_can_interface 
    {
        can_baud_10kbps = 0,
        can_baud_20kbps = 1,
        can_baud_50kbps = 2,
        can_baud_100kbps = 3,
        can_baud_125kbps = 4,
        can_baud_250kbps = 5,
        can_baud_500kbps = 6,
        can_baud_800kbps = 7,
        can_baud_1000kbps = 8
    };
}

#endif  // ROBAST_CAN_SERIAL_HELPER__CAN_SERIAL_HELPER_HPP_