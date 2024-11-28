#ifndef CAN__CAN_DB_HPP_
#define CAN__CAN_DB_HPP_

#include <vector>

#include "can/can_db_defines.hpp"
#include "can/can_message.hpp"

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
                       {CanSignal(can_signal::bit_start::drawer_unlock::MODULE_ID, can_signal::bit_length::drawer_unlock::MODULE_ID, 0),
                        CanSignal(can_signal::bit_start::drawer_unlock::DRAWER_ID, can_signal::bit_length::drawer_unlock::DRAWER_ID, 0)}),
            CanMessage(
                can_id::DRAWER_FEEDBACK,
                can_dlc::DRAWER_FEEDBACK,
                {CanSignal(can_signal::bit_start::drawer_feedback::MODULE_ID, can_signal::bit_length::drawer_feedback::MODULE_ID, 0),
                 CanSignal(can_signal::bit_start::drawer_feedback::DRAWER_ID, can_signal::bit_length::drawer_feedback::DRAWER_ID, 0),
                 CanSignal(can_signal::bit_start::drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED, can_signal::bit_length::drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED, 0),
                 CanSignal(can_signal::bit_start::drawer_feedback::IS_LOCK_SWITCH_PUSHED, can_signal::bit_length::drawer_feedback::IS_LOCK_SWITCH_PUSHED, 0)

                }),
            CanMessage(
                can_id::ELECTRICAL_DRAWER_TASK,
                can_dlc::ELECTRICAL_DRAWER_TASK,
                {CanSignal(can_signal::bit_start::e_drawer_task::MODULE_ID, can_signal::bit_length::e_drawer_task::MODULE_ID, 0),
                 CanSignal(can_signal::bit_start::e_drawer_task::DRAWER_ID, can_signal::bit_length::e_drawer_task::DRAWER_ID, 0),
                 CanSignal(can_signal::bit_start::e_drawer_task::DRAWER_TARGET_POSITION, can_signal::bit_length::e_drawer_task::DRAWER_TARGET_POSITION, 0),
                 CanSignal(can_signal::bit_start::e_drawer_task::DRAWER_SPEED, can_signal::bit_length::e_drawer_task::DRAWER_SPEED, 0),
                 CanSignal(
                     can_signal::bit_start::e_drawer_task::DRAWER_STALL_GUARD_VALUE, can_signal::bit_length::e_drawer_task::DRAWER_STALL_GUARD_VALUE, 0)}),
            CanMessage(
                can_id::ELECTRICAL_DRAWER_FEEDBACK,
                can_dlc::ELECTRICAL_DRAWER_FEEDBACK,
                {CanSignal(can_signal::bit_start::e_drawer_feedback::MODULE_ID, can_signal::bit_length::e_drawer_feedback::MODULE_ID, 0),
                 CanSignal(can_signal::bit_start::e_drawer_feedback::DRAWER_ID, can_signal::bit_length::e_drawer_feedback::DRAWER_ID, 0),
                 CanSignal(can_signal::bit_start::e_drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED, can_signal::bit_length::e_drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED, 0),
                 CanSignal(can_signal::bit_start::e_drawer_feedback::IS_LOCK_SWITCH_PUSHED, can_signal::bit_length::e_drawer_feedback::IS_LOCK_SWITCH_PUSHED, 0),
                 CanSignal(can_signal::bit_start::e_drawer_feedback::DRAWER_IS_STALL_GUARD_TRIGGERED,
                           can_signal::bit_length::e_drawer_feedback::DRAWER_IS_STALL_GUARD_TRIGGERED,
                           0),
                 CanSignal(can_signal::bit_start::e_drawer_feedback::DRAWER_POSITION, can_signal::bit_length::e_drawer_feedback::DRAWER_POSITION, 0),
                 CanSignal(can_signal::bit_start::e_drawer_feedback::IS_PUSH_TO_CLOSE_TRIGGERED, can_signal::bit_length::e_drawer_feedback::IS_PUSH_TO_CLOSE_TRIGGERED, 0)}),
            CanMessage(can_id::ERROR_FEEDBACK,
                       can_dlc::ERROR_FEEDBACK,
                       {CanSignal(can_signal::bit_start::error_feedback::MODULE_ID, can_signal::bit_length::error_feedback::MODULE_ID, 0),
                        CanSignal(can_signal::bit_start::error_feedback::DRAWER_ID, can_signal::bit_length::error_feedback::DRAWER_ID, 0),
                        CanSignal(can_signal::bit_start::error_feedback::ERROR_CODE, can_signal::bit_length::error_feedback::ERROR_CODE, 0)}),
            CanMessage(
                can_id::LED_HEADER,
                can_dlc::LED_HEADER,
                {CanSignal(can_signal::bit_start::led_header::MODULE_ID, can_signal::bit_length::led_header::MODULE_ID, 0),
                 CanSignal(can_signal::bit_start::led_header::START_INDEX, can_signal::bit_length::led_header::START_INDEX, 0),
                 CanSignal(can_signal::bit_start::led_header::NUM_OF_LEDS, can_signal::bit_length::led_header::NUM_OF_LEDS, 0),
                 CanSignal(
                     can_signal::bit_start::led_header::FADE_TIME_IN_HUNDREDS_OF_MS, can_signal::bit_length::led_header::FADE_TIME_IN_HUNDREDS_OF_MS, 0)}),
            CanMessage(
                can_id::LED_STATE,
                can_dlc::LED_STATE,
                {CanSignal(can_signal::bit_start::led_state::MODULE_ID, can_signal::bit_length::led_state::MODULE_ID, 0),
                 CanSignal(can_signal::bit_start::led_state::LED_STATE_RED, can_signal::bit_length::led_state::LED_STATE_RED, 0),
                 CanSignal(can_signal::bit_start::led_state::LED_STATE_GREEN, can_signal::bit_length::led_state::LED_STATE_GREEN, 0),
                 CanSignal(can_signal::bit_start::led_state::LED_STATE_BLUE, can_signal::bit_length::led_state::LED_STATE_BLUE, 0),
                 CanSignal(can_signal::bit_start::led_state::LED_STATE_BRIGHTNESS, can_signal::bit_length::led_state::LED_STATE_BRIGHTNESS, 0),
                 CanSignal(can_signal::bit_start::led_state::IS_GROUP_STATE, can_signal::bit_length::led_state::IS_GROUP_STATE, 0),
                  }),
            CanMessage(
                can_id::TRAY_LED_BRIGHTNESS,
                can_dlc::TRAY_LED_BRIGHTNESS,
                {CanSignal(can_signal::bit_start::tray_led_brightness::MODULE_ID, can_signal::bit_length::tray_led_brightness::MODULE_ID, 0),
                 CanSignal(can_signal::bit_start::tray_led_brightness::TRAY_ID, can_signal::bit_length::tray_led_brightness::TRAY_ID, 0),
                 CanSignal(can_signal::bit_start::tray_led_brightness::TRAY_LED_ROW_INDEX, can_signal::bit_length::tray_led_brightness::TRAY_LED_ROW_INDEX, 0),
                 CanSignal(can_signal::bit_start::tray_led_brightness::TRAY_LED_STATE_BRIGHNESS, can_signal::bit_length::tray_led_brightness::TRAY_LED_STATE_BRIGHNESS, 0)}),
            CanMessage(
                can_id::MODULE_CONFIG,
                can_dlc::MODULE_CONFIG,
                {CanSignal(can_signal::bit_start::module_config::MODULE_ID, can_signal::bit_length::module_config::MODULE_ID, 0),
                 CanSignal(can_signal::bit_start::module_config::CONFIG_ID, can_signal::bit_length::module_config::CONFIG_ID, 0),
                 CanSignal(can_signal::bit_start::module_config::CONFIG_VALUE, can_signal::bit_length::module_config::CONFIG_VALUE, 0)}),
            CanMessage(
                can_id::ELECTRICAL_DRAWER_MOTOR_CONTROL,
                can_dlc::ELECTRICAL_DRAWER_MOTOR_CONTROL,
                {CanSignal(can_signal::bit_start::electrical_drawer_motor_control::MODULE_ID, can_signal::bit_length::electrical_drawer_motor_control::MODULE_ID, 0),
                 CanSignal(can_signal::bit_start::electrical_drawer_motor_control::MOTOR_ID, can_signal::bit_length::electrical_drawer_motor_control::MOTOR_ID, 0),
                 CanSignal(can_signal::bit_start::electrical_drawer_motor_control::ENABLE_MOTOR, can_signal::bit_length::electrical_drawer_motor_control::ENABLE_MOTOR, 0),
                 CanSignal(can_signal::bit_start::electrical_drawer_motor_control::CONFIRM_CONTROL_CHANGE, can_signal::bit_length::electrical_drawer_motor_control::CONFIRM_CONTROL_CHANGE, 0)}),
            CanMessage(
                can_id::HEARTBEAT,
                can_dlc::HEARTBEAT,
                {CanSignal(can_signal::bit_start::heartbeat::MODULE_ID, can_signal::bit_length::heartbeat::MODULE_ID, 0),
                 CanSignal(can_signal::bit_start::heartbeat::INTERVAL_IN_MS, can_signal::bit_length::heartbeat::INTERVAL_IN_MS, 0)}
            )
        };
    };
} // namespace robast_can_msgs

#endif // CAN__CAN_DB_HPP_
