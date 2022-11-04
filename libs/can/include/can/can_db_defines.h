#ifndef CAN_DB_DEFINES_HPP_
#define CAN_DB_DEFINES_HPP_

namespace robast_can_msgs
{
    #define CAN_STD_MSG_DLC_MAXIMUM 8

    #define NUM_OF_CAN_MSGS 3

    /*********************************************************************************************************
     CAN IDs and DLCs for the CAN Bus
    *********************************************************************************************************/

    #define CAN_ID_DRAWER_LOCK 0x001
    #define CAN_ID_DRAWER_LED 0x002
    #define CAN_ID_DRAWER_FEEDBACK 0x003

    #define CAN_DLC_DRAWER_LOCK 4
    #define CAN_DLC_DRAWER_LED 8
    #define CAN_DLC_DRAWER_FEEDBACK 4

    /*********************************************************************************************************
     CAN msg index and can signal index to access the msg and signals in our can_db vector
    *********************************************************************************************************/

    #define CAN_MSG_DRAWER_LOCK  0
    #define CAN_MSG_DRAWER_LED 1
    #define CAN_MSG_DRAWER_FEEDBACK 2

    #define CAN_SIGNAL_DRAWER_CONTROLLER_ID 0
    #define CAN_SIGNAL_OPEN_LOCK_1 1
    #define CAN_SIGNAL_OPEN_LOCK_2 2

    #define CAN_SIGNAL_DRAWER_CONTROLLER_ID 0
    #define CAN_SIGNAL_LED_RED 1
    #define CAN_SIGNAL_LED_GREEN 2
    #define CAN_SIGNAL_LED_BLUE 3
    #define CAN_SIGNAL_LED_BRIGHTNESS 4
    #define CAN_SIGNAL_LED_MODE 5

    #define CAN_SIGNAL_DRAWER_CONTROLLER_ID 0
    #define CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED 1
    #define CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED 2
    #define CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED 3
    #define CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED 4

    /*********************************************************************************************************
     CAN SIGNAL BIT START
    *********************************************************************************************************/

    #define CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START 0
    #define CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH 24
    #define CAN_SIGNAL_OPEN_LOCK_1_BIT_START 24
    #define CAN_SIGNAL_OPEN_LOCK_1_BIT_LENGTH 1
    #define CAN_SIGNAL_OPEN_LOCK_2_BIT_START 25
    #define CAN_SIGNAL_OPEN_LOCK_2_BIT_LENGTH 1

    #define CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START 0
    #define CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH 24
    #define CAN_SIGNAL_LED_RED_BIT_START 24
    #define CAN_SIGNAL_LED_RED_BIT_LENGTH 8
    #define CAN_SIGNAL_LED_GREEN_BIT_START 32
    #define CAN_SIGNAL_LED_GREEN_BIT_LENGTH 8
    #define CAN_SIGNAL_LED_BLUE_BIT_START 40
    #define CAN_SIGNAL_LED_BLUE_BIT_LENGTH 8
    #define CAN_SIGNAL_LED_BRIGHTNESS_BIT_START 48
    #define CAN_SIGNAL_LED_BRIGHTNESS_BIT_LENGTH 8
    #define CAN_SIGNAL_LED_MODE_BIT_START 56
    #define CAN_SIGNAL_LED_MODE_BIT_LENGTH 3

    #define CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START 0
    #define CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH 24
    #define CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED_BIT_START 24
    #define CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED_BIT_LENGTH 1
    #define CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED_BIT_START 25
    #define CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED_BIT_LENGTH 1
    #define CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED_BIT_START 26
    #define CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED_BIT_LENGTH 1
    #define CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED_BIT_START 27
    #define CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED_BIT_LENGTH 1

    /*******************************************
     * Some defines for actual CAN data
    *******************************************/

    #define CAN_DATA_CLOSE_LOCK 0
    #define CAN_DATA_OPEN_LOCK 1

    #define CAN_DATA_SWITCH_IS_NOT_PUSHED 0
    #define CAN_DATA_SWITCH_IS_PUSHED 1
}

#endif /* CAN_DB_DEFINES_HPP_ */