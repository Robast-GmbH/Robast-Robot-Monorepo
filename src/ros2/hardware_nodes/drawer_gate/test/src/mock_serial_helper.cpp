#include "test/mock_serial_helper.hpp"
#include <iostream>
#include <fstream>

namespace serial_helper
{

    MockSerialHelper::MockSerialHelper()
    {}

    MockSerialHelper::~MockSerialHelper()
    {}

    string MockSerialHelper::open_serial()
    {
        return "";
    }

    void MockSerialHelper::close_serial()
    {}

    uint16_t MockSerialHelper::read_serial(string* result, uint16_t max_num_bytes)
    {
        robast_can_msgs::CanMessage can_message = robast_can_msgs::CanMessage(
            CAN_ID_DRAWER_FEEDBACK,
            CAN_DLC_DRAWER_FEEDBACK,
            {
                robast_can_msgs::CanSignal(CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START, CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH, 6),
                robast_can_msgs::CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED_BIT_LENGTH, 1),
                robast_can_msgs::CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED_BIT_LENGTH, 0),
                robast_can_msgs::CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED_BIT_LENGTH, 0),
                robast_can_msgs::CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED_BIT_LENGTH, 0),
            });
        robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

        std::optional<std::string> ascii_command = robast_can_msgs::encode_can_message_into_ascii_command(can_message, can_db.can_messages);

        *result = ascii_command.value();

        return 1;
    }

    string MockSerialHelper::write_serial(string msg)
    {
        return "";
    }

    string MockSerialHelper::send_ascii_cmd(string cmd)
    {
        return "";
    }

    string MockSerialHelper::ascii_interaction(string cmd, string* response, uint16_t response_size)
    {}

}