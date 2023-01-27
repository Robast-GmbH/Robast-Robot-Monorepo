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
                    CAN_ID_DRAWER_LOCK,
                    CAN_DLC_DRAWER_LOCK,
                    {
                        CanSignal(CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START, CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_OPEN_LOCK_1_BIT_START, CAN_SIGNAL_OPEN_LOCK_1_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_OPEN_LOCK_2_BIT_START, CAN_SIGNAL_OPEN_LOCK_2_BIT_LENGTH, 0),
                    }
                ),
                CanMessage(
                    CAN_ID_DRAWER_LED,
                    CAN_DLC_DRAWER_LED,
                    {
                        CanSignal(CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START, CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_LED_RED_BIT_START, CAN_SIGNAL_LED_RED_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_LED_GREEN_BIT_START, CAN_SIGNAL_LED_GREEN_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_LED_BLUE_BIT_START, CAN_SIGNAL_LED_BLUE_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_LED_BRIGHTNESS_BIT_START, CAN_SIGNAL_LED_BRIGHTNESS_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_LED_MODE_BIT_START, CAN_SIGNAL_LED_MODE_BIT_LENGTH, 0),
                    }
                ),
                CanMessage(
                    CAN_ID_DRAWER_FEEDBACK,
                    CAN_DLC_DRAWER_FEEDBACK,
                    {
                        CanSignal(CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START, CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED_BIT_LENGTH, 0)
                    }
                ),
                CanMessage(
                    CAN_ID_DOOR_MANIPULATOR,
                    CAN_DLC_DOOR_MANIPULATOR,
                    {
                        CanSignal(CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START, CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_MOTOR_ID_BIT_START, CAN_SIGNAL_MOTOR_ID_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_MOTOR_E1_BIT_START, CAN_SIGNAL_MOTOR_E1_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_MOTOR_E2_BIT_START, CAN_SIGNAL_MOTOR_E2_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_MOTOR_E3_BIT_START, CAN_SIGNAL_MOTOR_E3_BIT_LENGTH, 0),
                        CanSignal(CAN_SIGNAL_MOTOR_E4_BIT_START, CAN_SIGNAL_MOTOR_E4_BIT_LENGTH, 0),
                    }
                )
            };
    };
}  // namespace robast_can_msgs

#endif  // CAN__CAN_DB_HPP_
