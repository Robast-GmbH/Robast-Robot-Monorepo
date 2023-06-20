#ifndef CAN__CAN_DB_HPP_
#define CAN__CAN_DB_HPP_

#include <vector>

#include "can/can_message.h"
#include "can/can_db_defines.h"

namespace robast_can_msgs
{
    class CanDb
    {
    public:
        /**
         * @brief A constructor for robast_can_msgs::CanDb class
         */
        CanDb() = default;

        /**
         * @brief A destructor for robast_can_msgs::CanDb class
         */
        ~CanDb() = default;

        // Vector that contains all the CAN messages stored in the CanDb
        const std::vector<CanMessage> can_messages = {
            CanMessage(
                CAN_ID_DRAWER_UNLOCK,
                CAN_DLC_DRAWER_UNLOCK,
                {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0)}),
            CanMessage(
                CAN_ID_DRAWER_FEEDBACK,
                CAN_DLC_DRAWER_FEEDBACK,
                {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_LENGTH, 0)

                }),
            CanMessage(
                CAN_ID_DRAWER_LED,
                CAN_DLC_DRAWER_LED,
                {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_LED_RED_BIT_START, CAN_SIGNAL_LED_RED_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_LED_GREEN_BIT_START, CAN_SIGNAL_LED_GREEN_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_LED_BLUE_BIT_START, CAN_SIGNAL_LED_BLUE_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_LED_BRIGHTNESS_BIT_START, CAN_SIGNAL_LED_BRIGHTNESS_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_LED_MODE_BIT_START, CAN_SIGNAL_LED_MODE_BIT_LENGTH, 0)}),
            CanMessage(
                CAN_ID_ELECTRICAL_DRAWER_TASK,
                CAN_DLC_ELECTRIC_DRAWER_TASK,
                {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_TARGET_POSITION_BIT_START, CAN_SIGNAL_DRAWER_TARGET_POSITION_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_SPEED_BIT_START, CAN_SIGNAL_DRAWER_SPEED_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE_BIT_START, CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE_BIT_LENGTH, 0)}),
            CanMessage(
                CAN_ID_ELECTRICAL_DRAWER_FEEDBACK,
                CAN_DLC_ELECTRIC_DRAWER_FEEDBACK,
                {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED_BIT_START, CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED_BIT_LENGTH, 0),
                 CanSignal(CAN_SIGNAL_DRAWER_POSITION_BIT_START, CAN_SIGNAL_DRAWER_POSITION_BIT_LENGTH, 0)}),
        };
    };
} // namespace robast_can_msgs

#endif // CAN__CAN_DB_HPP_
