#ifndef CAN__CAN_DB_DEFINES_HPP_
#define CAN__CAN_DB_DEFINES_HPP_

#include <stdint.h>

namespace robast_can_msgs
{
#define CAN_STD_MSG_DLC_MAXIMUM 8

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
    namespace id
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

    } // id

    namespace bit_start
    {
      namespace drawer_unlock
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
      } // drawer_unlock

      namespace drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 32;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 33;
      } // drawer_feedback

      namespace e_drawer_task
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
        constexpr uint8_t DRAWER_TARGET_POSITION = 32;
        constexpr uint8_t DRAWER_SPEED = 40;
        constexpr uint8_t DRAWER_STALL_GUARD_VALUE = 48;
      } // e_drawer_task

      namespace e_drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 32;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 33;
        constexpr uint8_t DRAWER_IS_STALL_GUARD_TRIGGERED = 34;
        constexpr uint8_t DRAWER_POSITION = 35;
        constexpr uint8_t IS_PUSH_TO_CLOSE_TRIGGERED = 43;
      } // e_drawer_feedback

      namespace error_feedback
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
        constexpr uint8_t ERROR_CODE = 32;
      } // error_feedback

      namespace led_header
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t START_INDEX = 24;
        constexpr uint8_t NUM_OF_LEDS = 40;
        constexpr uint8_t FADE_TIME_IN_HUNDREDS_OF_MS = 56;
      } // led_header

      namespace single_led
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t LED_STATE_RED = 24;
        constexpr uint8_t LED_STATE_GREEN = 32;
        constexpr uint8_t LED_STATE_BLUE = 40;
        constexpr uint8_t LED_STATE_BRIGHTNESS = 48;
      } // single_led

      namespace tray_led_brightness
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t TRAY_ID = 24;
        constexpr uint8_t TRAY_LED_ROW_INDEX = 32;
        constexpr uint8_t TRAY_LED_STATE_BRIGHNESS = 40;
      } // tray_led_brightness

      namespace module_config
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t CONFIG_ID = 24;
        constexpr uint8_t CONFIG_VALUE = 32;
      } // module_config
    } // bit_config

    namespace bit_length
    {
      namespace drawer_unlock
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
      } // drawer_unlock

      namespace drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 1;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 1;
      } // drawer_feedback

      namespace e_drawer_task
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
        constexpr uint8_t DRAWER_TARGET_POSITION = 8;
        constexpr uint8_t DRAWER_SPEED = 8;
        constexpr uint8_t DRAWER_STALL_GUARD_VALUE = 8;
      } // e_drawer_task

      namespace e_drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 1;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 1;
        constexpr uint8_t DRAWER_IS_STALL_GUARD_TRIGGERED = 1;
        constexpr uint8_t DRAWER_POSITION = 8;
        constexpr uint8_t IS_PUSH_TO_CLOSE_TRIGGERED = 1;
      } // e_drawer_feedback

      namespace error_feedback
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
        constexpr uint8_t ERROR_CODE = 6;
      } // bit_length

      namespace led_header
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t START_INDEX = 16;
        constexpr uint8_t NUM_OF_LEDS = 16;
        constexpr uint8_t FADE_TIME_IN_HUNDREDS_OF_MS = 8;
      } // led_header

      namespace single_led
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t LED_STATE_RED = 8;
        constexpr uint8_t LED_STATE_GREEN = 8;
        constexpr uint8_t LED_STATE_BLUE = 8;
        constexpr uint8_t LED_STATE_BRIGHTNESS = 8;
      } // single_led

      namespace tray_led_brightness
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t TRAY_ID = 8;
        constexpr uint8_t TRAY_LED_ROW_INDEX = 8;
        constexpr uint8_t TRAY_LED_STATE_BRIGHNESS = 8;
      } // tray_led_brightness

      namespace module_config
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t CONFIG_ID = 8;
        constexpr uint8_t CONFIG_VALUE = 32;
      } // module_config

    } // bit_length

  } // can_signal

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

#endif // CAN__CAN_DB_DEFINES_HPP_
