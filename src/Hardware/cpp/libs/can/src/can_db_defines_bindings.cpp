#include <pybind11/pybind11.h>

#include "can/can_db_defines.hpp"

namespace py = pybind11;

PYBIND11_MODULE(can_db_defines_bindings, m)
{
  // Expose can_id constants
  m.attr("CAN_ID_DRAWER_UNLOCK") = robast_can_msgs::can_id::DRAWER_UNLOCK;
  m.attr("CAN_ID_DRAWER_FEEDBACK") = robast_can_msgs::can_id::DRAWER_FEEDBACK;
  m.attr("CAN_ID_ELECTRICAL_DRAWER_TASK") = robast_can_msgs::can_id::ELECTRICAL_DRAWER_TASK;
  m.attr("CAN_ID_ELECTRICAL_DRAWER_FEEDBACK") = robast_can_msgs::can_id::ELECTRICAL_DRAWER_FEEDBACK;
  m.attr("CAN_ID_ERROR_FEEDBACK") = robast_can_msgs::can_id::ERROR_FEEDBACK;
  m.attr("CAN_ID_LED_HEADER") = robast_can_msgs::can_id::LED_HEADER;
  m.attr("CAN_ID_SINGLE_LED_STATE") = robast_can_msgs::can_id::SINGLE_LED_STATE;
  m.attr("CAN_ID_TRAY_LED_BRIGHTNESS") = robast_can_msgs::can_id::TRAY_LED_BRIGHTNESS;
  m.attr("CAN_ID_MODULE_CONFIG") = robast_can_msgs::can_id::MODULE_CONFIG;
  m.attr("CAN_ID_ELECTRICAL_DRAWER_MOTOR_CONTROL") = robast_can_msgs::can_id::ELECTRICAL_DRAWER_MOTOR_CONTROL;

  // Expose can_dlc constants
  m.attr("DLC_DRAWER_UNLOCK") = robast_can_msgs::can_dlc::DRAWER_UNLOCK;
  m.attr("DLC_DRAWER_FEEDBACK") = robast_can_msgs::can_dlc::DRAWER_FEEDBACK;
  m.attr("DLC_ELECTRICAL_DRAWER_TASK") = robast_can_msgs::can_dlc::ELECTRICAL_DRAWER_TASK;
  m.attr("DLC_ELECTRICAL_DRAWER_FEEDBACK") = robast_can_msgs::can_dlc::ELECTRICAL_DRAWER_FEEDBACK;
  m.attr("DLC_ERROR_FEEDBACK") = robast_can_msgs::can_dlc::ERROR_FEEDBACK;
  m.attr("DLC_LED_HEADER") = robast_can_msgs::can_dlc::LED_HEADER;
  m.attr("DLC_SINGLE_LED_STATE") = robast_can_msgs::can_dlc::SINGLE_LED_STATE;
  m.attr("DLC_TRAY_LED_BRIGHTNESS") = robast_can_msgs::can_dlc::TRAY_LED_BRIGHTNESS;
  m.attr("DLC_MODULE_CONFIG") = robast_can_msgs::can_dlc::MODULE_CONFIG;
  m.attr("DLC_ELECTRICAL_DRAWER_MOTOR_CONTROL") = robast_can_msgs::can_dlc::ELECTRICAL_DRAWER_MOTOR_CONTROL;
}