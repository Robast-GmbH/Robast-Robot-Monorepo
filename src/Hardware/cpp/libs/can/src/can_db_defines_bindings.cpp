#include <pybind11/pybind11.h>

#include "can/can_db_defines.hpp"

namespace py = pybind11;

PYBIND11_MODULE(can_db_defines_bindings, m)
{
  // Expose can_id constants
  py::module_ can_id = m.def_submodule("can_id");
  can_id.attr("DRAWER_UNLOCK") = robast_can_msgs::can_id::DRAWER_UNLOCK;
  can_id.attr("DRAWER_FEEDBACK") = robast_can_msgs::can_id::DRAWER_FEEDBACK;
  can_id.attr("ELECTRICAL_DRAWER_TASK") = robast_can_msgs::can_id::ELECTRICAL_DRAWER_TASK;
  can_id.attr("ELECTRICAL_DRAWER_FEEDBACK") = robast_can_msgs::can_id::ELECTRICAL_DRAWER_FEEDBACK;
  can_id.attr("ERROR_FEEDBACK") = robast_can_msgs::can_id::ERROR_FEEDBACK;
  can_id.attr("LED_HEADER") = robast_can_msgs::can_id::LED_HEADER;
  can_id.attr("SINGLE_LED_STATE") = robast_can_msgs::can_id::SINGLE_LED_STATE;
  can_id.attr("TRAY_LED_BRIGHTNESS") = robast_can_msgs::can_id::TRAY_LED_BRIGHTNESS;
  can_id.attr("MODULE_CONFIG") = robast_can_msgs::can_id::MODULE_CONFIG;
  can_id.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL") = robast_can_msgs::can_id::ELECTRICAL_DRAWER_MOTOR_CONTROL;

  // Expose can_dlc constants
  py::module_ can_dlc = m.def_submodule("can_dlc");
  can_dlc.attr("DRAWER_UNLOCK") = robast_can_msgs::can_dlc::DRAWER_UNLOCK;
  can_dlc.attr("DRAWER_FEEDBACK") = robast_can_msgs::can_dlc::DRAWER_FEEDBACK;
  can_dlc.attr("ELECTRICAL_DRAWER_TASK") = robast_can_msgs::can_dlc::ELECTRICAL_DRAWER_TASK;
  can_dlc.attr("ELECTRICAL_DRAWER_FEEDBACK") = robast_can_msgs::can_dlc::ELECTRICAL_DRAWER_FEEDBACK;
  can_dlc.attr("ERROR_FEEDBACK") = robast_can_msgs::can_dlc::ERROR_FEEDBACK;
  can_dlc.attr("LED_HEADER") = robast_can_msgs::can_dlc::LED_HEADER;
  can_dlc.attr("SINGLE_LED_STATE") = robast_can_msgs::can_dlc::SINGLE_LED_STATE;
  can_dlc.attr("TRAY_LED_BRIGHTNESS") = robast_can_msgs::can_dlc::TRAY_LED_BRIGHTNESS;
  can_dlc.attr("MODULE_CONFIG") = robast_can_msgs::can_dlc::MODULE_CONFIG;
  can_dlc.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL") = robast_can_msgs::can_dlc::ELECTRICAL_DRAWER_MOTOR_CONTROL;

  // Expose can_signal constants
  py::module_ can_signal = m.def_submodule("can_signal");

  py::module_ id = can_signal.def_submodule("id");
  id.attr("DRAWER_UNLOCK_MODULE_ID") = robast_can_msgs::can_signal::id::drawer_unlock::MODULE_ID;
  id.attr("DRAWER_UNLOCK_DRAWER_ID") = robast_can_msgs::can_signal::id::drawer_unlock::DRAWER_ID;

  id.attr("DRAWER_FEEDBACK_MODULE_ID") = robast_can_msgs::can_signal::id::drawer_feedback::MODULE_ID;
  id.attr("DRAWER_FEEDBACK_DRAWER_ID") = robast_can_msgs::can_signal::id::drawer_feedback::DRAWER_ID;
  id.attr("DRAWER_FEEDBACK_IS_ENDSTOP_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::id::drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED;
  id.attr("DRAWER_FEEDBACK_IS_LOCK_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::id::drawer_feedback::IS_LOCK_SWITCH_PUSHED;

  id.attr("E_DRAWER_TASK_MODULE_ID") = robast_can_msgs::can_signal::id::e_drawer_task::MODULE_ID;
  id.attr("E_DRAWER_TASK_DRAWER_ID") = robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_ID;
  id.attr("E_DRAWER_TASK_DRAWER_TARGET_POSITION") =
    robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_TARGET_POSITION;
  id.attr("E_DRAWER_TASK_DRAWER_SPEED") = robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_SPEED;
  id.attr("E_DRAWER_TASK_DRAWER_STALL_GUARD_VALUE") =
    robast_can_msgs::can_signal::id::e_drawer_task::DRAWER_STALL_GUARD_VALUE;

  id.attr("E_DRAWER_FEEDBACK_MODULE_ID") = robast_can_msgs::can_signal::id::e_drawer_feedback::MODULE_ID;
  id.attr("E_DRAWER_FEEDBACK_DRAWER_ID") = robast_can_msgs::can_signal::id::e_drawer_feedback::DRAWER_ID;
  id.attr("E_DRAWER_FEEDBACK_IS_ENDSTOP_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::id::e_drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED;
  id.attr("E_DRAWER_FEEDBACK_IS_LOCK_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::id::e_drawer_feedback::IS_LOCK_SWITCH_PUSHED;
  id.attr("E_DRAWER_FEEDBACK_DRAWER_IS_STALL_GUARD_TRIGGERED") =
    robast_can_msgs::can_signal::id::e_drawer_feedback::DRAWER_IS_STALL_GUARD_TRIGGERED;
  id.attr("E_DRAWER_FEEDBACK_DRAWER_POSITION") = robast_can_msgs::can_signal::id::e_drawer_feedback::DRAWER_POSITION;
  id.attr("E_DRAWER_FEEDBACK_IS_PUSH_TO_CLOSE_TRIGGERED") =
    robast_can_msgs::can_signal::id::e_drawer_feedback::IS_PUSH_TO_CLOSE_TRIGGERED;

  id.attr("ERROR_FEEDBACK_MODULE_ID") = robast_can_msgs::can_signal::id::error_feedback::MODULE_ID;
  id.attr("ERROR_FEEDBACK_DRAWER_ID") = robast_can_msgs::can_signal::id::error_feedback::DRAWER_ID;
  id.attr("ERROR_FEEDBACK_ERROR_CODE") = robast_can_msgs::can_signal::id::error_feedback::ERROR_CODE;

  id.attr("LED_HEADER_MODULE_ID") = robast_can_msgs::can_signal::id::led_header::MODULE_ID;
  id.attr("LED_HEADER_START_INDEX") = robast_can_msgs::can_signal::id::led_header::START_INDEX;
  id.attr("LED_HEADER_NUM_OF_LEDS") = robast_can_msgs::can_signal::id::led_header::NUM_OF_LEDS;
  id.attr("LED_HEADER_FADE_TIME_IN_HUNDREDS_OF_MS") =
    robast_can_msgs::can_signal::id::led_header::FADE_TIME_IN_HUNDREDS_OF_MS;

  id.attr("SINGLE_LED_MODULE_ID") = robast_can_msgs::can_signal::id::single_led::MODULE_ID;
  id.attr("SINGLE_LED_LED_STATE_RED") = robast_can_msgs::can_signal::id::single_led::LED_STATE_RED;
  id.attr("SINGLE_LED_LED_STATE_GREEN") = robast_can_msgs::can_signal::id::single_led::LED_STATE_GREEN;
  id.attr("SINGLE_LED_LED_STATE_BLUE") = robast_can_msgs::can_signal::id::single_led::LED_STATE_BLUE;
  id.attr("SINGLE_LED_LED_STATE_BRIGHTNESS") = robast_can_msgs::can_signal::id::single_led::LED_STATE_BRIGHTNESS;

  id.attr("TRAY_LED_BRIGHTNESS_MODULE_ID") = robast_can_msgs::can_signal::id::tray_led_brightness::MODULE_ID;
  id.attr("TRAY_LED_BRIGHTNESS_TRAY_ID") = robast_can_msgs::can_signal::id::tray_led_brightness::TRAY_ID;
  id.attr("TRAY_LED_BRIGHTNESS_TRAY_LED_ROW_INDEX") =
    robast_can_msgs::can_signal::id::tray_led_brightness::TRAY_LED_ROW_INDEX;
  id.attr("TRAY_LED_BRIGHTNESS_TRAY_LED_STATE_BRIGHNESS") =
    robast_can_msgs::can_signal::id::tray_led_brightness::TRAY_LED_STATE_BRIGHNESS;

  id.attr("MODULE_CONFIG_MODULE_ID") = robast_can_msgs::can_signal::id::module_config::MODULE_ID;
  id.attr("MODULE_CONFIG_CONFIG_ID") = robast_can_msgs::can_signal::id::module_config::CONFIG_ID;
  id.attr("MODULE_CONFIG_CONFIG_VALUE") = robast_can_msgs::can_signal::id::module_config::CONFIG_VALUE;

  id.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL_MODULE_ID") =
    robast_can_msgs::can_signal::id::electrical_drawer_motor_control::MODULE_ID;
  id.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL_MOTOR_ID") =
    robast_can_msgs::can_signal::id::electrical_drawer_motor_control::MOTOR_ID;
  id.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL_ENABLE_MOTOR") =
    robast_can_msgs::can_signal::id::electrical_drawer_motor_control::ENABLE_MOTOR;

  py::module_ bit_start = can_signal.def_submodule("bit_start");
  bit_start.attr("DRAWER_UNLOCK_MODULE_ID") = robast_can_msgs::can_signal::bit_start::drawer_unlock::MODULE_ID;
  bit_start.attr("DRAWER_UNLOCK_DRAWER_ID") = robast_can_msgs::can_signal::bit_start::drawer_unlock::DRAWER_ID;

  bit_start.attr("DRAWER_FEEDBACK_MODULE_ID") = robast_can_msgs::can_signal::bit_start::drawer_feedback::MODULE_ID;
  bit_start.attr("DRAWER_FEEDBACK_DRAWER_ID") = robast_can_msgs::can_signal::bit_start::drawer_feedback::DRAWER_ID;
  bit_start.attr("DRAWER_FEEDBACK_IS_ENDSTOP_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::bit_start::drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED;
  bit_start.attr("DRAWER_FEEDBACK_IS_LOCK_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::bit_start::drawer_feedback::IS_LOCK_SWITCH_PUSHED;

  bit_start.attr("E_DRAWER_TASK_MODULE_ID") = robast_can_msgs::can_signal::bit_start::e_drawer_task::MODULE_ID;
  bit_start.attr("E_DRAWER_TASK_DRAWER_ID") = robast_can_msgs::can_signal::bit_start::e_drawer_task::DRAWER_ID;
  bit_start.attr("E_DRAWER_TASK_DRAWER_TARGET_POSITION") =
    robast_can_msgs::can_signal::bit_start::e_drawer_task::DRAWER_TARGET_POSITION;
  bit_start.attr("E_DRAWER_TASK_DRAWER_SPEED") = robast_can_msgs::can_signal::bit_start::e_drawer_task::DRAWER_SPEED;
  bit_start.attr("E_DRAWER_TASK_DRAWER_STALL_GUARD_VALUE") =
    robast_can_msgs::can_signal::bit_start::e_drawer_task::DRAWER_STALL_GUARD_VALUE;

  bit_start.attr("E_DRAWER_FEEDBACK_MODULE_ID") = robast_can_msgs::can_signal::bit_start::e_drawer_feedback::MODULE_ID;
  bit_start.attr("E_DRAWER_FEEDBACK_DRAWER_ID") = robast_can_msgs::can_signal::bit_start::e_drawer_feedback::DRAWER_ID;
  bit_start.attr("E_DRAWER_FEEDBACK_IS_ENDSTOP_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::bit_start::e_drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED;
  bit_start.attr("E_DRAWER_FEEDBACK_IS_LOCK_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::bit_start::e_drawer_feedback::IS_LOCK_SWITCH_PUSHED;
  bit_start.attr("E_DRAWER_FEEDBACK_DRAWER_IS_STALL_GUARD_TRIGGERED") =
    robast_can_msgs::can_signal::bit_start::e_drawer_feedback::DRAWER_IS_STALL_GUARD_TRIGGERED;
  bit_start.attr("E_DRAWER_FEEDBACK_DRAWER_POSITION") =
    robast_can_msgs::can_signal::bit_start::e_drawer_feedback::DRAWER_POSITION;
  bit_start.attr("E_DRAWER_FEEDBACK_IS_PUSH_TO_CLOSE_TRIGGERED") =
    robast_can_msgs::can_signal::bit_start::e_drawer_feedback::IS_PUSH_TO_CLOSE_TRIGGERED;

  bit_start.attr("ERROR_FEEDBACK_MODULE_ID") = robast_can_msgs::can_signal::bit_start::error_feedback::MODULE_ID;
  bit_start.attr("ERROR_FEEDBACK_DRAWER_ID") = robast_can_msgs::can_signal::bit_start::error_feedback::DRAWER_ID;
  bit_start.attr("ERROR_FEEDBACK_ERROR_CODE") = robast_can_msgs::can_signal::bit_start::error_feedback::ERROR_CODE;

  bit_start.attr("LED_HEADER_MODULE_ID") = robast_can_msgs::can_signal::bit_start::led_header::MODULE_ID;
  bit_start.attr("LED_HEADER_START_INDEX") = robast_can_msgs::can_signal::bit_start::led_header::START_INDEX;
  bit_start.attr("LED_HEADER_NUM_OF_LEDS") = robast_can_msgs::can_signal::bit_start::led_header::NUM_OF_LEDS;
  bit_start.attr("LED_HEADER_FADE_TIME_IN_HUNDREDS_OF_MS") =
    robast_can_msgs::can_signal::bit_start::led_header::FADE_TIME_IN_HUNDREDS_OF_MS;

  bit_start.attr("SINGLE_LED_MODULE_ID") = robast_can_msgs::can_signal::bit_start::single_led::MODULE_ID;
  bit_start.attr("SINGLE_LED_LED_STATE_RED") = robast_can_msgs::can_signal::bit_start::single_led::LED_STATE_RED;
  bit_start.attr("SINGLE_LED_LED_STATE_GREEN") = robast_can_msgs::can_signal::bit_start::single_led::LED_STATE_GREEN;
  bit_start.attr("SINGLE_LED_LED_STATE_BLUE") = robast_can_msgs::can_signal::bit_start::single_led::LED_STATE_BLUE;
  bit_start.attr("SINGLE_LED_LED_STATE_BRIGHTNESS") =
    robast_can_msgs::can_signal::bit_start::single_led::LED_STATE_BRIGHTNESS;

  bit_start.attr("TRAY_LED_BRIGHTNESS_MODULE_ID") =
    robast_can_msgs::can_signal::bit_start::tray_led_brightness::MODULE_ID;
  bit_start.attr("TRAY_LED_BRIGHTNESS_TRAY_ID") = robast_can_msgs::can_signal::bit_start::tray_led_brightness::TRAY_ID;
  bit_start.attr("TRAY_LED_BRIGHTNESS_TRAY_LED_ROW_INDEX") =
    robast_can_msgs::can_signal::bit_start::tray_led_brightness::TRAY_LED_ROW_INDEX;
  bit_start.attr("TRAY_LED_BRIGHTNESS_TRAY_LED_STATE_BRIGHNESS") =
    robast_can_msgs::can_signal::bit_start::tray_led_brightness::TRAY_LED_STATE_BRIGHNESS;

  bit_start.attr("MODULE_CONFIG_MODULE_ID") = robast_can_msgs::can_signal::bit_start::module_config::MODULE_ID;
  bit_start.attr("MODULE_CONFIG_CONFIG_ID") = robast_can_msgs::can_signal::bit_start::module_config::CONFIG_ID;
  bit_start.attr("MODULE_CONFIG_CONFIG_VALUE") = robast_can_msgs::can_signal::bit_start::module_config::CONFIG_VALUE;

  bit_start.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL_MODULE_ID") =
    robast_can_msgs::can_signal::bit_start::electrical_drawer_motor_control::MODULE_ID;
  bit_start.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL_MOTOR_ID") =
    robast_can_msgs::can_signal::bit_start::electrical_drawer_motor_control::MOTOR_ID;
  bit_start.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL_ENABLE_MOTOR") =
    robast_can_msgs::can_signal::bit_start::electrical_drawer_motor_control::ENABLE_MOTOR;

  py::module_ bit_length = can_signal.def_submodule("bit_length");
  bit_length.attr("DRAWER_UNLOCK_MODULE_ID") = robast_can_msgs::can_signal::bit_length::drawer_unlock::MODULE_ID;
  bit_length.attr("DRAWER_UNLOCK_DRAWER_ID") = robast_can_msgs::can_signal::bit_length::drawer_unlock::DRAWER_ID;

  bit_length.attr("DRAWER_FEEDBACK_MODULE_ID") = robast_can_msgs::can_signal::bit_length::drawer_feedback::MODULE_ID;
  bit_length.attr("DRAWER_FEEDBACK_DRAWER_ID") = robast_can_msgs::can_signal::bit_length::drawer_feedback::DRAWER_ID;
  bit_length.attr("DRAWER_FEEDBACK_IS_ENDSTOP_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::bit_length::drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED;
  bit_length.attr("DRAWER_FEEDBACK_IS_LOCK_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::bit_length::drawer_feedback::IS_LOCK_SWITCH_PUSHED;

  bit_length.attr("E_DRAWER_TASK_MODULE_ID") = robast_can_msgs::can_signal::bit_length::e_drawer_task::MODULE_ID;
  bit_length.attr("E_DRAWER_TASK_DRAWER_ID") = robast_can_msgs::can_signal::bit_length::e_drawer_task::DRAWER_ID;
  bit_length.attr("E_DRAWER_TASK_DRAWER_TARGET_POSITION") =
    robast_can_msgs::can_signal::bit_length::e_drawer_task::DRAWER_TARGET_POSITION;
  bit_length.attr("E_DRAWER_TASK_DRAWER_SPEED") = robast_can_msgs::can_signal::bit_length::e_drawer_task::DRAWER_SPEED;
  bit_length.attr("E_DRAWER_TASK_DRAWER_STALL_GUARD_VALUE") =
    robast_can_msgs::can_signal::bit_length::e_drawer_task::DRAWER_STALL_GUARD_VALUE;

  bit_length.attr("E_DRAWER_FEEDBACK_MODULE_ID") =
    robast_can_msgs::can_signal::bit_length::e_drawer_feedback::MODULE_ID;
  bit_length.attr("E_DRAWER_FEEDBACK_DRAWER_ID") =
    robast_can_msgs::can_signal::bit_length::e_drawer_feedback::DRAWER_ID;
  bit_length.attr("E_DRAWER_FEEDBACK_IS_ENDSTOP_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::bit_length::e_drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED;
  bit_length.attr("E_DRAWER_FEEDBACK_IS_LOCK_SWITCH_PUSHED") =
    robast_can_msgs::can_signal::bit_length::e_drawer_feedback::IS_LOCK_SWITCH_PUSHED;
  bit_length.attr("E_DRAWER_FEEDBACK_DRAWER_IS_STALL_GUARD_TRIGGERED") =
    robast_can_msgs::can_signal::bit_length::e_drawer_feedback::DRAWER_IS_STALL_GUARD_TRIGGERED;
  bit_length.attr("E_DRAWER_FEEDBACK_DRAWER_POSITION") =
    robast_can_msgs::can_signal::bit_length::e_drawer_feedback::DRAWER_POSITION;
  bit_length.attr("E_DRAWER_FEEDBACK_IS_PUSH_TO_CLOSE_TRIGGERED") =
    robast_can_msgs::can_signal::bit_length::e_drawer_feedback::IS_PUSH_TO_CLOSE_TRIGGERED;

  bit_length.attr("ERROR_FEEDBACK_MODULE_ID") = robast_can_msgs::can_signal::bit_length::error_feedback::MODULE_ID;
  bit_length.attr("ERROR_FEEDBACK_DRAWER_ID") = robast_can_msgs::can_signal::bit_length::error_feedback::DRAWER_ID;
  bit_length.attr("ERROR_FEEDBACK_ERROR_CODE") = robast_can_msgs::can_signal::bit_length::error_feedback::ERROR_CODE;

  bit_length.attr("LED_HEADER_MODULE_ID") = robast_can_msgs::can_signal::bit_length::led_header::MODULE_ID;
  bit_length.attr("LED_HEADER_START_INDEX") = robast_can_msgs::can_signal::bit_length::led_header::START_INDEX;
  bit_length.attr("LED_HEADER_NUM_OF_LEDS") = robast_can_msgs::can_signal::bit_length::led_header::NUM_OF_LEDS;
  bit_length.attr("LED_HEADER_FADE_TIME_IN_HUNDREDS_OF_MS") =
    robast_can_msgs::can_signal::bit_length::led_header::FADE_TIME_IN_HUNDREDS_OF_MS;

  bit_length.attr("SINGLE_LED_MODULE_ID") = robast_can_msgs::can_signal::bit_length::single_led::MODULE_ID;
  bit_length.attr("SINGLE_LED_LED_STATE_RED") = robast_can_msgs::can_signal::bit_length::single_led::LED_STATE_RED;
  bit_length.attr("SINGLE_LED_LED_STATE_GREEN") = robast_can_msgs::can_signal::bit_length::single_led::LED_STATE_GREEN;
  bit_length.attr("SINGLE_LED_LED_STATE_BLUE") = robast_can_msgs::can_signal::bit_length::single_led::LED_STATE_BLUE;
  bit_length.attr("SINGLE_LED_LED_STATE_BRIGHTNESS") =
    robast_can_msgs::can_signal::bit_length::single_led::LED_STATE_BRIGHTNESS;

  bit_length.attr("TRAY_LED_BRIGHTNESS_MODULE_ID") =
    robast_can_msgs::can_signal::bit_length::tray_led_brightness::MODULE_ID;
  bit_length.attr("TRAY_LED_BRIGHTNESS_TRAY_ID") =
    robast_can_msgs::can_signal::bit_length::tray_led_brightness::TRAY_ID;
  bit_length.attr("TRAY_LED_BRIGHTNESS_TRAY_LED_ROW_INDEX") =
    robast_can_msgs::can_signal::bit_length::tray_led_brightness::TRAY_LED_ROW_INDEX;
  bit_length.attr("TRAY_LED_BRIGHTNESS_TRAY_LED_STATE_BRIGHNESS") =
    robast_can_msgs::can_signal::bit_length::tray_led_brightness::TRAY_LED_STATE_BRIGHNESS;

  bit_length.attr("MODULE_CONFIG_MODULE_ID") = robast_can_msgs::can_signal::bit_length::module_config::MODULE_ID;
  bit_length.attr("MODULE_CONFIG_CONFIG_ID") = robast_can_msgs::can_signal::bit_length::module_config::CONFIG_ID;
  bit_length.attr("MODULE_CONFIG_CONFIG_VALUE") = robast_can_msgs::can_signal::bit_length::module_config::CONFIG_VALUE;

  bit_length.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL_MODULE_ID") =
    robast_can_msgs::can_signal::bit_length::electrical_drawer_motor_control::MODULE_ID;
  bit_length.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL_MOTOR_ID") =
    robast_can_msgs::can_signal::bit_length::electrical_drawer_motor_control::MOTOR_ID;
  bit_length.attr("ELECTRICAL_DRAWER_MOTOR_CONTROL_ENABLE_MOTOR") =
    robast_can_msgs::can_signal::bit_length::electrical_drawer_motor_control::ENABLE_MOTOR;
}