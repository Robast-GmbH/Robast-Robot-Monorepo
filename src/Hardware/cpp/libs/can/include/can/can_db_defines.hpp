#ifndef CAN__CAN_DB_DEFINES_HPP_
#define CAN__CAN_DB_DEFINES_HPP_

#include <stdint.h>

namespace robast_can_msgs
{
  constexpr uint8_t CAN_STD_MSG_DLC_MAXIMUM = 8;

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
    constexpr uint16_t ELECTRICAL_DRAWER_MOTOR_CONTROL = 0x00A;
  }   // namespace can_id

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
    constexpr uint8_t ELECTRICAL_DRAWER_MOTOR_CONTROL = 4;
  }   // namespace can_dlc

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
    constexpr uint8_t ELECTRICAL_DRAWER_MOTOR_CONTROL = 9;
  }   // namespace can_msg

  namespace can_signal
  {
    namespace id
    {
      namespace drawer_unlock
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 1;
      }   // namespace drawer_unlock

      namespace drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 1;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 2;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 3;
      }   // namespace drawer_feedback

      namespace e_drawer_task
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 1;
        constexpr uint8_t DRAWER_TARGET_POSITION = 2;
        constexpr uint8_t DRAWER_SPEED = 3;
        constexpr uint8_t DRAWER_STALL_GUARD_VALUE = 4;
      }   // namespace e_drawer_task

      namespace e_drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 1;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 2;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 3;
        constexpr uint8_t DRAWER_IS_STALL_GUARD_TRIGGERED = 4;
        constexpr uint8_t DRAWER_POSITION = 5;
        constexpr uint8_t IS_PUSH_TO_CLOSE_TRIGGERED = 6;
      }   // namespace e_drawer_feedback

      namespace error_feedback
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 1;
        constexpr uint8_t ERROR_CODE = 2;
      }   // namespace error_feedback

      namespace led_header
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t START_INDEX = 1;
        constexpr uint8_t NUM_OF_LEDS = 2;
        constexpr uint8_t FADE_TIME_IN_HUNDREDS_OF_MS = 3;
      }   // namespace led_header

      namespace single_led
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t LED_STATE_RED = 1;
        constexpr uint8_t LED_STATE_GREEN = 2;
        constexpr uint8_t LED_STATE_BLUE = 3;
        constexpr uint8_t LED_STATE_BRIGHTNESS = 4;
      }   // namespace single_led

      namespace tray_led_brightness
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t TRAY_ID = 1;
        constexpr uint8_t TRAY_LED_ROW_INDEX = 2;
        constexpr uint8_t TRAY_LED_STATE_BRIGHNESS = 3;
      }   // namespace tray_led_brightness

      namespace module_config
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t CONFIG_ID = 1;
        constexpr uint8_t CONFIG_VALUE = 2;
      }   // namespace module_config

      namespace electrical_drawer_motor_control
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t MOTOR_ID = 1;
        constexpr uint8_t ENABLE_MOTOR = 2;
      }   // namespace electrical_drawer_motor_control

    }   // namespace id

    namespace bit_start
    {
      namespace drawer_unlock
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
      }   // namespace drawer_unlock

      namespace drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 32;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 33;
      }   // namespace drawer_feedback

      namespace e_drawer_task
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
        constexpr uint8_t DRAWER_TARGET_POSITION = 32;
        constexpr uint8_t DRAWER_SPEED = 40;
        constexpr uint8_t DRAWER_STALL_GUARD_VALUE = 48;
      }   // namespace e_drawer_task

      namespace e_drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 32;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 33;
        constexpr uint8_t DRAWER_IS_STALL_GUARD_TRIGGERED = 34;
        constexpr uint8_t DRAWER_POSITION = 35;
        constexpr uint8_t IS_PUSH_TO_CLOSE_TRIGGERED = 43;
      }   // namespace e_drawer_feedback

      namespace error_feedback
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t DRAWER_ID = 24;
        constexpr uint8_t ERROR_CODE = 32;
      }   // namespace error_feedback

      namespace led_header
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t START_INDEX = 24;
        constexpr uint8_t NUM_OF_LEDS = 40;
        constexpr uint8_t FADE_TIME_IN_HUNDREDS_OF_MS = 56;
      }   // namespace led_header

      namespace single_led
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t LED_STATE_RED = 24;
        constexpr uint8_t LED_STATE_GREEN = 32;
        constexpr uint8_t LED_STATE_BLUE = 40;
        constexpr uint8_t LED_STATE_BRIGHTNESS = 48;
      }   // namespace single_led

      namespace tray_led_brightness
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t TRAY_ID = 24;
        constexpr uint8_t TRAY_LED_ROW_INDEX = 32;
        constexpr uint8_t TRAY_LED_STATE_BRIGHNESS = 40;
      }   // namespace tray_led_brightness

      namespace module_config
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t CONFIG_ID = 24;
        constexpr uint8_t CONFIG_VALUE = 32;
      }   // namespace module_config

      namespace electrical_drawer_motor_control
      {
        constexpr uint8_t MODULE_ID = 0;
        constexpr uint8_t MOTOR_ID = 24;
        constexpr uint8_t ENABLE_MOTOR = 26;
      }   // namespace electrical_drawer_motor_control
    }   // namespace bit_start

    namespace bit_length
    {
      namespace drawer_unlock
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
      }   // namespace drawer_unlock

      namespace drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 1;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 1;
      }   // namespace drawer_feedback

      namespace e_drawer_task
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
        constexpr uint8_t DRAWER_TARGET_POSITION = 8;
        constexpr uint8_t DRAWER_SPEED = 8;
        constexpr uint8_t DRAWER_STALL_GUARD_VALUE = 8;
      }   // namespace e_drawer_task

      namespace e_drawer_feedback
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
        constexpr uint8_t IS_ENDSTOP_SWITCH_PUSHED = 1;
        constexpr uint8_t IS_LOCK_SWITCH_PUSHED = 1;
        constexpr uint8_t DRAWER_IS_STALL_GUARD_TRIGGERED = 1;
        constexpr uint8_t DRAWER_POSITION = 8;
        constexpr uint8_t IS_PUSH_TO_CLOSE_TRIGGERED = 1;
      }   // namespace e_drawer_feedback

      namespace error_feedback
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t DRAWER_ID = 8;
        constexpr uint8_t ERROR_CODE = 6;
      }   // namespace error_feedback

      namespace led_header
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t START_INDEX = 16;
        constexpr uint8_t NUM_OF_LEDS = 16;
        constexpr uint8_t FADE_TIME_IN_HUNDREDS_OF_MS = 8;
      }   // namespace led_header

      namespace single_led
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t LED_STATE_RED = 8;
        constexpr uint8_t LED_STATE_GREEN = 8;
        constexpr uint8_t LED_STATE_BLUE = 8;
        constexpr uint8_t LED_STATE_BRIGHTNESS = 8;
      }   // namespace single_led

      namespace tray_led_brightness
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t TRAY_ID = 8;
        constexpr uint8_t TRAY_LED_ROW_INDEX = 8;
        constexpr uint8_t TRAY_LED_STATE_BRIGHNESS = 8;
      }   // namespace tray_led_brightness

      namespace module_config
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t CONFIG_ID = 8;
        constexpr uint8_t CONFIG_VALUE = 32;
      }   // namespace module_config

      namespace electrical_drawer_motor_control
      {
        constexpr uint8_t MODULE_ID = 24;
        constexpr uint8_t MOTOR_ID = 2;
        constexpr uint8_t ENABLE_MOTOR = 1;
      }   // namespace electrical_drawer_motor_control

    }   // namespace bit_length

  }   // namespace can_signal
  namespace can_data
  {
    constexpr uint64_t SWITCH_IS_NOT_PUSHED = 0;
    constexpr uint64_t SWITCH_IS_PUSHED = 1;
    constexpr uint64_t STALL_GUARD_TRIGGERED = 1;
    constexpr uint64_t PUSH_TO_CLOSE_TRIGGERED = 1;
    constexpr uint64_t DISABLE_MOTOR = 0;
    constexpr uint64_t ENABLE_MOTOR = 1;

    namespace error_code
    {
      constexpr uint64_t TIMEOUT_DRAWER_NOT_OPENED = 1;
      constexpr uint64_t DRAWER_CLOSED_IN_IDLE_STATE = 2;
      constexpr uint64_t E_DRAWER_TASK_NOT_SUPPORTED_BY_MODULE = 3;
      constexpr uint64_t MOTOR_DRIVER_STATE_CONTROL_NOT_SUPPORTED_BY_MODULE = 4;
    }   // namespace error_code
  }   // namespace can_data
}   // namespace robast_can_msgs

#endif   // CAN__CAN_DB_DEFINES_HPP_
