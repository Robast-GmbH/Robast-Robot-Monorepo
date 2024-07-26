#ifndef CAN__CAN_DB_HPP_
#define CAN__CAN_DB_HPP_

#include <vector>

#include "can/can_db_defines.h"
#include "can/can_message.h"

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
        CanMessage(can_id::DRAWER_UNLOCK,
                   can_dlc::DRAWER_UNLOCK,
                   {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
                    CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0)}),
        CanMessage(
            can_id::DRAWER_FEEDBACK,
            can_dlc::DRAWER_FEEDBACK,
            {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_LENGTH, 0)

            }),
        CanMessage(
            can_id::ELECTRICAL_DRAWER_TASK,
            can_dlc::ELECTRICAL_DRAWER_TASK,
            {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_DRAWER_TARGET_POSITION_BIT_START, CAN_SIGNAL_DRAWER_TARGET_POSITION_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_DRAWER_SPEED_BIT_START, CAN_SIGNAL_DRAWER_SPEED_BIT_LENGTH, 0),
             CanSignal(
                 CAN_SIGNAL_DRAWER_STALL_GUARD_VALUE_BIT_START, CAN_SIGNAL_DRAWER_STALL_GUARD_VALUE_BIT_LENGTH, 0)}),
        CanMessage(
            can_id::ELECTRICAL_DRAWER_FEEDBACK,
            can_dlc::ELECTRICAL_DRAWER_FEEDBACK,
            {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED_BIT_START,
                       CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED_BIT_LENGTH,
                       0),
             CanSignal(CAN_SIGNAL_DRAWER_POSITION_BIT_START, CAN_SIGNAL_DRAWER_POSITION_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_IS_PUSH_TO_CLOSE_TRIGGERED_BIT_START, CAN_SIGNAL_IS_PUSH_TO_CLOSE_TRIGGERED_BIT_LENGTH, 0)}),
        CanMessage(can_id::ERROR_FEEDBACK,
                   can_dlc::ERROR_FEEDBACK,
                   {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
                    CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, 0),
                    CanSignal(CAN_SIGNAL_ERROR_CODE_BIT_START, CAN_SIGNAL_ERROR_CODE_BIT_LENGTH, 0)}),
        CanMessage(
            can_id::LED_HEADER,
            can_dlc::LED_HEADER,
            {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_LED_START_INDEX_BIT_START, CAN_SIGNAL_LED_START_INDEX_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_NUM_OF_LEDS_BIT_START, CAN_SIGNAL_NUM_OF_LEDS_BIT_LENGTH, 0),
             CanSignal(
                 CAN_SIGNAL_FADE_TIME_IN_HUNDREDS_OF_MS_BIT_START, CAN_SIGNAL_FADE_TIME_IN_HUNDREDS_OF_MS_BIT_LENGTH, 0)}),
        CanMessage(
            can_id::SINGLE_LED_STATE,
            can_dlc::SINGLE_LED_STATE,
            {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_SINGLE_LED_STATE_RED_BIT_START, CAN_SIGNAL_SINGLE_LED_STATE_RED_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_SINGLE_LED_STATE_GREEN_BIT_START, CAN_SIGNAL_SINGLE_LED_STATE_GREEN_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_SINGLE_LED_STATE_BLUE_BIT_START, CAN_SIGNAL_SINGLE_LED_STATE_BLUE_BIT_LENGTH, 0),
             CanSignal(
                 CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS_BIT_START, CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS_BIT_LENGTH, 0)}),
        CanMessage(
            can_id::TRAY_LED_BRIGHTNESS,
            can_dlc::TRAY_LED_BRIGHTNESS,
            {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_TRAY_ID_BIT_START, CAN_SIGNAL_TRAY_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_TRAY_LED_ROW_INDEX_BIT_START, CAN_SIGNAL_TRAY_LED_ROW_INDEX_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_TRAY_LED_STATE_BRIGHNESS_BIT_START, CAN_SIGNAL_TRAY_LED_STATE_BRIGHNESS_BIT_LENGTH, 0)}),
        CanMessage(
            can_id::MODULE_CONFIG,
            can_dlc::MODULE_CONFIG,
            {CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_CONFIG_ID_BIT_START, CAN_SIGNAL_CONFIG_ID_BIT_LENGTH, 0),
             CanSignal(CAN_SIGNAL_CONFIG_VALUE_BIT_START, CAN_SIGNAL_CONFIG_VALUE_BIT_LENGTH, 0)}),
    };
  };
} // namespace robast_can_msgs

#endif // CAN__CAN_DB_HPP_
