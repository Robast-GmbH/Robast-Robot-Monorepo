#ifndef CAN__CAN_DB_DEFINES_H_
#define CAN__CAN_DB_DEFINES_H_

namespace robast_can_msgs
{
#define CAN_STD_MSG_DLC_MAXIMUM 8

#define NUM_OF_CAN_MSGS 5

  /*********************************************************************************************************
   CAN IDs and DLCs for the CAN Bus
  *********************************************************************************************************/

#define CAN_ID_DRAWER_UNLOCK              0x001
#define CAN_ID_DRAWER_FEEDBACK            0x002
#define CAN_ID_DRAWER_LED                 0x003
#define CAN_ID_ELECTRICAL_DRAWER_TASK     0x004
#define CAN_ID_ELECTRICAL_DRAWER_FEEDBACK 0x005
#define CAN_ID_ERROR_FEEDBACK             0x006

#define CAN_DLC_DRAWER_UNLOCK            4
#define CAN_DLC_DRAWER_FEEDBACK          4
#define CAN_DLC_DRAWER_LED               8
#define CAN_DLC_ELECTRIC_DRAWER_TASK     6
#define CAN_DLC_ELECTRIC_DRAWER_FEEDBACK 5
#define CAN_DLC_ERROR_FEEDBACK           4

  /*********************************************************************************************************
   CAN msg index and can signal index to access the msg and signals in our can_db vector
  *********************************************************************************************************/

#define CAN_MSG_DRAWER_UNLOCK              0
#define CAN_MSG_DRAWER_FEEDBACK            1
#define CAN_MSG_DRAWER_LED                 2
#define CAN_MSG_ELECTRICAL_DRAWER_TASK     3
#define CAN_MSG_ELECTRICAL_DRAWER_FEEDBACK 4
#define CAN_MSG_ERROR_FEEDBACK             5

// DRAWER_UNLOCK
#define CAN_SIGNAL_MODULE_ID 0
#define CAN_SIGNAL_DRAWER_ID 1

// DRAWER_FEEDBACK
#define CAN_SIGNAL_MODULE_ID                0
#define CAN_SIGNAL_DRAWER_ID                1
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED 2
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED    3

// DRAWER_LED
#define CAN_SIGNAL_MODULE_ID      0
#define CAN_SIGNAL_DRAWER_ID      1
#define CAN_SIGNAL_LED_RED        2
#define CAN_SIGNAL_LED_GREEN      3
#define CAN_SIGNAL_LED_BLUE       4
#define CAN_SIGNAL_LED_BRIGHTNESS 5
#define CAN_SIGNAL_LED_MODE       6

// ELECTRICAL_DRAWER_TASK
#define CAN_SIGNAL_MODULE_ID                 0
#define CAN_SIGNAL_DRAWER_ID                 1
#define CAN_SIGNAL_DRAWER_TARGET_POSITION    2
#define CAN_SIGNAL_DRAWER_SPEED              3
#define CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE 4

// ELECTRICAL_DRAWER_FEEDBACK
#define CAN_SIGNAL_MODULE_ID                       0
#define CAN_SIGNAL_DRAWER_ID                       1
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED        2
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED           3
#define CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED 4
#define CAN_SIGNAL_DRAWER_POSITION                 5

// ERROR_FEEDBACK
#define CAN_SIGNAL_MODULE_ID  0
#define CAN_SIGNAL_DRAWER_ID  1
#define CAN_SIGNAL_ERROR_CODE 2

/*********************************************************************************************************
 CAN SIGNAL BIT START
*********************************************************************************************************/

// DRAWER_UNLOCK
#define CAN_SIGNAL_MODULE_ID_BIT_START  0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH 24
#define CAN_SIGNAL_DRAWER_ID_BIT_START  24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH 2

// DRAWER_FEEDBACK
#define CAN_SIGNAL_MODULE_ID_BIT_START                 0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH                24
#define CAN_SIGNAL_DRAWER_ID_BIT_START                 24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH                2
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_START  26
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_LENGTH 1
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_START     27
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_LENGTH    1

// DRAWER_LED
#define CAN_SIGNAL_MODULE_ID_BIT_START       0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH      24
#define CAN_SIGNAL_DRAWER_ID_BIT_START       24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH      2
#define CAN_SIGNAL_LED_RED_BIT_START         26
#define CAN_SIGNAL_LED_RED_BIT_LENGTH        8
#define CAN_SIGNAL_LED_GREEN_BIT_START       34
#define CAN_SIGNAL_LED_GREEN_BIT_LENGTH      8
#define CAN_SIGNAL_LED_BLUE_BIT_START        42
#define CAN_SIGNAL_LED_BLUE_BIT_LENGTH       8
#define CAN_SIGNAL_LED_BRIGHTNESS_BIT_START  50
#define CAN_SIGNAL_LED_BRIGHTNESS_BIT_LENGTH 8
#define CAN_SIGNAL_LED_MODE_BIT_START        58
#define CAN_SIGNAL_LED_MODE_BIT_LENGTH       3

// ELECTRICAL_DRAWER_TASK
#define CAN_SIGNAL_MODULE_ID_BIT_START                  0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH                 24
#define CAN_SIGNAL_DRAWER_ID_BIT_START                  24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH                 2
#define CAN_SIGNAL_DRAWER_TARGET_POSITION_BIT_START     26
#define CAN_SIGNAL_DRAWER_TARGET_POSITION_BIT_LENGTH    8
#define CAN_SIGNAL_DRAWER_SPEED_BIT_START               34
#define CAN_SIGNAL_DRAWER_SPEED_BIT_LENGTH              8
#define CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE_BIT_START  42
#define CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE_BIT_LENGTH 1

// ELECTRICAL_DRAWER_FEEDBACK
#define CAN_SIGNAL_MODULE_ID_BIT_START                        0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH                       24
#define CAN_SIGNAL_DRAWER_ID_BIT_START                        24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH                       2
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_START         26
#define CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED_BIT_LENGTH        1
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_START            27
#define CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED_BIT_LENGTH           1
#define CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED_BIT_START  28
#define CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED_BIT_LENGTH 1
#define CAN_SIGNAL_DRAWER_POSITION_BIT_START                  29
#define CAN_SIGNAL_DRAWER_POSITION_BIT_LENGTH                 8

// ERROR FEEDBACK
#define CAN_SIGNAL_MODULE_ID_BIT_START   0
#define CAN_SIGNAL_MODULE_ID_BIT_LENGTH  24
#define CAN_SIGNAL_DRAWER_ID_BIT_START   24
#define CAN_SIGNAL_DRAWER_ID_BIT_LENGTH  2
#define CAN_SIGNAL_ERROR_CODE_BIT_START  26
#define CAN_SIGNAL_ERROR_CODE_BIT_LENGTH 6

  /*******************************************
   * Some defines for actual CAN data
   *******************************************/

#define CAN_DATA_SWITCH_IS_NOT_PUSHED 0
#define CAN_DATA_SWITCH_IS_PUSHED     1

#define CAN_DATA_ELECTRICAL_DRAWER_STALL_GUARD_DISABLED 0
#define CAN_DATA_ELECTRICAL_DRAWER_STALL_GUARD_ENABLED  1

#define CAN_DATA_ELECTRICAL_DRAWER_IS_STALL_GUARD_NOT_TRIGGERED 0
#define CAN_DATA_ELECTRICAL_DRAWER_IS_STALL_GUARD_TRIGGERED     1

#define CAN_DATA_ERROR_CODE_TIMEOUT_DRAWER_NOT_OPENED 1

}   // namespace robast_can_msgs

#endif   // CAN__CAN_DB_DEFINES_H_
