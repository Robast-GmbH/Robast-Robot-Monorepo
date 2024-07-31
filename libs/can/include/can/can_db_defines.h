#ifndef CAN__CAN_DB_DEFINES_H_
#define CAN__CAN_DB_DEFINES_H_

#include <stdint.h>

namespace robast_can_msgs
{
#define CAN_STD_MSG_DLC_MAXIMUM 8

#define NUM_OF_CAN_MSGS 8

  /*********************************************************************************************************
   CAN IDs and DLCs for the CAN Bus
  *********************************************************************************************************/

  namespace can_id
  {
    constexpr uint16_t DRAWER_UNLOCK = 0x001;
    constexpr uint16_t DRAWER_FEEDBACK = 0x002;
    constexpr uint16_t ELECTRICAL_DRAWER_TASK = 0x003;
    constexpr uint16_t ELECTRICAL_DRAWER_FEEDBACK = 0x004;
    constexpr uint16_t ERROR_FEEDBACK = 0x005;
    constexpr uint16_t LED_HEADER = 0x006;
    constexpr uint16_t SINGLE_LED_STATE = 0x007;
    constexpr uint16_t TRAY_LED_BRIGHTNESS = 0x008;
    constexpr uint16_t MODULE_CONFIG = 0x009;
  }

  namespace can_dlc
  {
    constexpr uint8_t DRAWER_UNLOCK = 4;
    constexpr uint8_t DRAWER_FEEDBACK = 5;
    constexpr uint8_t ELECTRICAL_DRAWER_TASK = 7;
    constexpr uint8_t ELECTRICAL_DRAWER_FEEDBACK = 6;
    constexpr uint8_t ERROR_FEEDBACK = 5;
    constexpr uint8_t LED_HEADER = 8;
    constexpr uint8_t SINGLE_LED_STATE = 7;
    constexpr uint8_t TRAY_LED_BRIGHTNESS = 6;
    constexpr uint8_t MODULE_CONFIG = 8;
  }

  /*********************************************************************************************************
   CAN msg index and can signal index to access the msg and signals in our can_db vector
  *********************************************************************************************************/

  namespace can_msg
  {
    constexpr uint8_t DRAWER_UNLOCK = 0;
    constexpr uint8_t DRAWER_FEEDBACK = 1;
    constexpr uint8_t ELECTRICAL_DRAWER_TASK = 2;
    constexpr uint8_t ELECTRICAL_DRAWER_FEEDBACK = 3;
    constexpr uint8_t ERROR_FEEDBACK = 4;
    constexpr uint8_t LED_HEADER = 5;
    constexpr uint8_t SINGLE_LED_STATE = 6;
    constexpr uint8_t TRAY_LED_BRIGHTNESS = 7;
    constexpr uint8_t MODULE_CONFIG = 8;
  }

  namespace can_signal
  {
    namespace drawer_unlock
    {
      constexpr uint8_t MODULE_ID = 0;
      constexpr uint8_t DRAWER_ID = 1;
    } // drawer_unlock

    namespace drawer_feedback
    {
      constexpr uint8_t MODULE_ID = 0;
      constexpr uint8_t DRAWER_ID = 1;
      constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 2;
      constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 3;
    } // drawer_feedback

    namespace e_drawer_task
    {
      constexpr uint8_t MODULE_ID = 0;
      constexpr uint8_t DRAWER_ID = 1;
      constexpr uint8_t DRAWER_TARGET_POSITION = 2;
      constexpr uint8_t DRAWER_SPEED = 3;
      constexpr uint8_t DRAWER_STALL_GUARD_VALUE = 4;
    } // e_drawer_task

    namespace e_drawer_feedback
    {
      constexpr uint8_t MODULE_ID = 0;
      constexpr uint8_t DRAWER_ID = 1;
      constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 2;
      constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 3;
      constexpr uint8_t DRAWER_IS_STALL_GUARD_TRIGGERED = 4;
      constexpr uint8_t DRAWER_POSITION = 5;
      constexpr uint8_t IS_PUSH_TO_CLOSE_TRIGGERED = 6;
    } // e_drawer_feedback

    namespace error_feedback
    {
      constexpr uint8_t MODULE_ID = 0;
      constexpr uint8_t DRAWER_ID = 1;
      constexpr uint8_t ERROR_CODE = 2;
    } // error_feedback

    namespace led_header
    {
      constexpr uint8_t MODULE_ID = 0;
      constexpr uint8_t START_INDEX = 1;
      constexpr uint8_t NUM_OF_LEDS = 2;
      constexpr uint8_t FADE_TIME_IN_HUNDREDS_OF_MS = 3;
    } // led_header

    namespace single_led
    {
      constexpr uint8_t MODULE_ID = 0;
      constexpr uint8_t LED_STATE_RED = 1;
      constexpr uint8_t LED_STATE_GREEN = 2;
      constexpr uint8_t LED_STATE_BLUE = 3;
      constexpr uint8_t LED_STATE_BRIGHTNESS = 4;
    } // single_led

    namespace tray_led_brightness
    {
      constexpr uint8_t MODULE_ID = 0;
      constexpr uint8_t TRAY_ID = 1;
      constexpr uint8_t TRAY_LED_ROW_INDEX = 2;
      constexpr uint8_t TRAY_LED_STATE_BRIGHNESS = 3;
    } // tray_led_brightness

    namespace module_config
    {
      constexpr uint8_t MODULE_ID = 0;
      constexpr uint8_t CONFIG_ID = 1;
      constexpr uint8_t CONFIG_VALUE = 2;
    } // module_config

  } // can_signal

/*********************************************************************************************************
 CAN SIGNAL BIT START
*********************************************************************************************************/

// DRAWER_UNLOCK
#define CAN_SIGNAL_MODULE_ID_BIT_START 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_DRAWER_ID_BIT_START 24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH 8

// DRAWER_FEEDBACK
#define CAN_SIGNAL_MODULE_ID_BIT_START 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_DRAWER_ID_BIT_START 24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH 8
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_START 32
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_LENGTH 1
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_START 33
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_LENGTH 1

// ELECTRICAL_DRAWER_TASK
#define CAN_SIGNAL_MODULE_ID_BIT_START 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_DRAWER_ID_BIT_START 24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH 8
#define CAN_SIGNAL_DRAWER_TARGET_POSITION_BIT_START 32
#define CAN_SIGNAL_DRAWER_TARGET_POSITION_BIT_LENGTH 8
#define CAN_SIGNAL_DRAWER_SPEED_BIT_START 40
#define CAN_SIGNAL_DRAWER_SPEED_BIT_LENGTH 8
#define CAN_SIGNAL_DRAWER_STALL_GUARD_VALUE_BIT_START 48
#define CAN_SIGNAL_DRAWER_STALL_GUARD_VALUE_BIT_LENGTH 8

// ELECTRICAL_DRAWER_FEEDBACK
#define CAN_SIGNAL_MODULE_ID_BIT_START 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_DRAWER_ID_BIT_START 24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH 8
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_START 32
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_LENGTH 1
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_START 33
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_LENGTH 1
#define CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED_BIT_START 34
#define CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED_BIT_LENGTH 1
#define CAN_SIGNAL_DRAWER_POSITION_BIT_START 35
#define CAN_SIGNAL_DRAWER_POSITION_BIT_LENGTH 8
#define CAN_SIGNAL_IS_PUSH_TO_CLOSE_TRIGGERED_BIT_START 43
#define CAN_SIGNAL_IS_PUSH_TO_CLOSE_TRIGGERED_BIT_LENGTH 1

// ERROR FEEDBACK
#define CAN_SIGNAL_MODULE_ID_BIT_START 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_DRAWER_ID_BIT_START 24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH 8
#define CAN_SIGNAL_ERROR_CODE_BIT_START 32
#define CAN_SIGNAL_ERROR_CODE_BIT_LENGTH 6

// LED HEADER
#define CAN_SIGNAL_MODULE_ID_BIT_START 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_LED_START_INDEX_BIT_START 24
#define CAN_SIGNAL_LED_START_INDEX_BIT_LENGTH 16
#define CAN_SIGNAL_NUM_OF_LEDS_BIT_START 40
#define CAN_SIGNAL_NUM_OF_LEDS_BIT_LENGTH 16
#define CAN_SIGNAL_FADE_TIME_IN_HUNDREDS_OF_MS_BIT_START 56
#define CAN_SIGNAL_FADE_TIME_IN_HUNDREDS_OF_MS_BIT_LENGTH 8

// SINGLE_LED_STATE
#define CAN_SIGNAL_MODULE_ID_BIT_START 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_SINGLE_LED_STATE_RED_BIT_START 24
#define CAN_SIGNAL_SINGLE_LED_STATE_RED_BIT_LENGTH 8
#define CAN_SIGNAL_SINGLE_LED_STATE_GREEN_BIT_START 32
#define CAN_SIGNAL_SINGLE_LED_STATE_GREEN_BIT_LENGTH 8
#define CAN_SIGNAL_SINGLE_LED_STATE_BLUE_BIT_START 40
#define CAN_SIGNAL_SINGLE_LED_STATE_BLUE_BIT_LENGTH 8
#define CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS_BIT_START 48
#define CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS_BIT_LENGTH 8

// TRAY_LED_BRIGHTNESS
#define CAN_SIGNAL_MODULE_ID_BIT_START 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_TRAY_ID_BIT_START 24
#define CAN_SIGNAL_TRAY_ID_BIT_LENGTH 8
#define CAN_SIGNAL_TRAY_LED_ROW_INDEX_BIT_START 32
#define CAN_SIGNAL_TRAY_LED_ROW_INDEX_BIT_LENGTH 8
#define CAN_SIGNAL_TRAY_LED_STATE_BRIGHNESS_BIT_START 40
#define CAN_SIGNAL_TRAY_LED_STATE_BRIGHNESS_BIT_LENGTH 8

// MODULE_CONFIG
#define CAN_SIGNAL_MODULE_ID_BIT_START 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_CONFIG_ID_BIT_START 24
#define CAN_SIGNAL_CONFIG_ID_BIT_LENGTH 8
#define CAN_SIGNAL_CONFIG_VALUE_BIT_START 32
#define CAN_SIGNAL_CONFIG_VALUE_BIT_LENGTH 32

  /*******************************************
   * Some defines for actual CAN data
   *******************************************/

#define CAN_DATA_SWITCH_IS_NOT_PUSHED 0
#define CAN_DATA_SWITCH_IS_PUSHED 1

#define CAN_DATA_ELECTRICAL_DRAWER_STALL_GUARD_DISABLED 0

#define CAN_DATA_ELECTRICAL_DRAWER_IS_STALL_GUARD_NOT_TRIGGERED 0
#define CAN_DATA_ELECTRICAL_DRAWER_IS_STALL_GUARD_TRIGGERED 1

#define CAN_DATA_ERROR_CODE_TIMEOUT_DRAWER_NOT_OPENED 1

#define CAN_DATA_PUSH_TO_CLOSE_NOT_TRIGGERED 0
#define CAN_DATA_PUSH_TO_CLOSE_TRIGGERED 1

} // namespace robast_can_msgs

#endif // CAN__CAN_DB_DEFINES_H_
