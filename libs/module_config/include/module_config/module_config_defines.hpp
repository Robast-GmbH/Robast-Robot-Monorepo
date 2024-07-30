#ifndef MODULE_CONFIG__MODULE_CONFIG_DEFINES_HPP
#define MODULE_CONFIG__MODULE_CONFIG_DEFINES_HPP

#include <cstdint>

namespace module_config
{

  // Define the mapping struct
  template <int ID>
  struct ModuleConfigDataType;
  // Usage example:
  // using DataType = ModuleConfigDataType<drawer::MAX_SPEED>::type;
  // DataType value = 10;

  /********************************************************************************************************
   * E-Drawer configs
   *********************************************************************************************************/

  namespace drawer
  {
    constexpr uint8_t MAX_SPEED = 1;
    constexpr uint8_t HOMING_SPEED = 2;
    constexpr uint8_t INITIAL_HOMING_SPEED = 3;
    constexpr uint8_t MOVING_IN_DECELERATION_DISTANCE = 4;
    constexpr uint8_t MOVING_IN_FINAL_HOMING_DISTANCE = 5;
    constexpr uint8_t MOVING_OUT_DECELERATION_DISTANCE = 6;
    constexpr uint8_t PUSH_IN_AUTO_CLOSE_SPEED = 7;
    constexpr uint8_t PUSH_IN_AUTO_CLOSE_TMC_STALL_GUARD_VALUE = 8;
    constexpr uint8_t PUSH_IN_WAIT_TIME_AFTER_STALL_GUARD_TRIGGERED_IN_MS = 9;
    constexpr uint8_t USE_TMC_STALL_GUARD = 10;
  }   // namespace drawer

  namespace encoder
  {
    constexpr uint8_t OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT = 20;
    constexpr uint8_t ENCODER_COUNT_DRAWER_MAX_EXTENT = 21;
    constexpr uint8_t DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN = 22;
    constexpr uint8_t DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT = 23;
    constexpr uint8_t DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS = 24;
  }   // namespace encoder

  namespace motor_monitor
  {
    constexpr uint8_t ACTIVE_SPEED_THRESHOLD = 30;
    constexpr uint8_t LOWER_POSITION_THRESHOLD = 31;
    constexpr uint8_t MAX_TIME_DIFF_BETWEEN_ENCODER_MEASUREMENTS_IN_MS = 32;
    constexpr uint8_t SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL = 33;
  }   // namespace motor_monitor

  template <>
  struct ModuleConfigDataType<drawer::MAX_SPEED>
  {
    using type = uint32_t;
    static constexpr type default_value = 35000;
  };

  template <>
  struct ModuleConfigDataType<drawer::HOMING_SPEED>
  {
    using type = uint32_t;
    static constexpr type default_value = 500;
  };

  template <>
  struct ModuleConfigDataType<drawer::INITIAL_HOMING_SPEED>
  {
    using type = uint32_t;
    static constexpr type default_value = 1500;
  };

  template <>
  struct ModuleConfigDataType<drawer::MOVING_IN_DECELERATION_DISTANCE>
  {
    using type = uint8_t;
    static constexpr type default_value = 50;
  };

  template <>
  struct ModuleConfigDataType<drawer::MOVING_IN_FINAL_HOMING_DISTANCE>
  {
    using type = uint8_t;
    static constexpr type default_value = 1;
  };

  template <>
  struct ModuleConfigDataType<drawer::MOVING_OUT_DECELERATION_DISTANCE>
  {
    using type = uint8_t;
    static constexpr type default_value = 70;
  };

  template <>
  struct ModuleConfigDataType<drawer::PUSH_IN_AUTO_CLOSE_SPEED>
  {
    using type = uint8_t;
    static constexpr type default_value = 120;
  };

  template <>
  struct ModuleConfigDataType<drawer::PUSH_IN_AUTO_CLOSE_TMC_STALL_GUARD_VALUE>
  {
    using type = uint8_t;
    static constexpr type default_value = 75;
  };

  template <>
  struct ModuleConfigDataType<drawer::PUSH_IN_WAIT_TIME_AFTER_STALL_GUARD_TRIGGERED_IN_MS>
  {
    using type = uint32_t;
    static constexpr type default_value = 200;
  };

  template <>
  struct ModuleConfigDataType<drawer::USE_TMC_STALL_GUARD>
  {
    using type = bool;
    static constexpr type default_value = false;
  };

  /********************************************************************************************************
   * Configs for the encoder
   *********************************************************************************************************/

  template <>
  struct ModuleConfigDataType<encoder::OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT>
  {
    using type = uint32_t;
    static constexpr type default_value = 85000;
  };

  template <>
  struct ModuleConfigDataType<encoder::ENCODER_COUNT_DRAWER_MAX_EXTENT>
  {
    using type = uint32_t;
    static constexpr type default_value = 83000;
  };

  template <>
  struct ModuleConfigDataType<encoder::DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN>
  {
    using type = uint32_t;
    static constexpr type default_value = 1000;
  };

  template <>
  struct ModuleConfigDataType<encoder::DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT>
  {
    using type = float;
    static constexpr type default_value = 0.005;
  };

  template <>
  struct ModuleConfigDataType<encoder::DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS>
  {
    using type = uint32_t;
    static constexpr type default_value = 500;
  };

  /********************************************************************************************************
   * Configs for the motor monitor
   *********************************************************************************************************/

  template <>
  struct ModuleConfigDataType<motor_monitor::ACTIVE_SPEED_THRESHOLD>
  {
    using type = uint32_t;
    static constexpr type default_value = 800;
  };

  template <>
  struct ModuleConfigDataType<motor_monitor::LOWER_POSITION_THRESHOLD>
  {
    using type = uint32_t;
    // Right after the drawer is homed, there can be a tension in the belt which can cause the motor to move a bit
    // We don't want to trigger the stall guard in this case, so define a threshold from which to start monitoring
    static constexpr type default_value = 200;
  };

  template <>
  struct ModuleConfigDataType<motor_monitor::MAX_TIME_DIFF_BETWEEN_ENCODER_MEASUREMENTS_IN_MS>
  {
    using type = uint32_t;
    // usually one control loop takes 30-40ms (as of 29.07.2024)
    static constexpr type default_value = 100;
  };

  template <>
  struct ModuleConfigDataType<motor_monitor::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL>
  {
    using type = float;
    static constexpr type default_value = 0.35;
  };

}   // namespace module_config

#endif   // MODULE_CONFIG__MODULE_CONFIG_DEFINES_HPP