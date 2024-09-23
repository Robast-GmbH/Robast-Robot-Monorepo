#ifndef MODULE_CONFIG__MODULE_CONFIG_DEFINES_HPP
#define MODULE_CONFIG__MODULE_CONFIG_DEFINES_HPP

#include <cstdint>

namespace module_config
{

  // Define the mapping struct
  template <uint8_t ID>
  struct ModuleSetting;
  // Usage example:
  // using DataType = ModuleSetting<drawer::MAX_SPEED>::type;
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
    constexpr uint8_t MOVING_OUT_FINAL_SPEED = 7;
    constexpr uint8_t PUSH_IN_AUTO_CLOSE_SPEED = 8;
    constexpr uint8_t PUSH_IN_AUTO_CLOSE_TMC_STALL_GUARD_VALUE = 9;
    constexpr uint8_t PUSH_IN_WAIT_TIME_AFTER_STALL_GUARD_TRIGGERED_IN_MS = 10;
    constexpr uint8_t STALL_GUARD_WAIT_TIME_AFTER_MOVEMENT_STARTED_IN_MS = 11;
    constexpr uint8_t USE_TMC_STALL_GUARD = 12;
    constexpr uint8_t USE_MOTOR_MONITOR_STALL_GUARD = 13;
    constexpr uint8_t DRAWER_DEFAULT_ACCELERATION = 14;
  } // namespace drawer

  namespace encoder
  {
    constexpr uint8_t OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT = 20;
    constexpr uint8_t ENCODER_COUNT_DRAWER_MAX_EXTENT = 21;
    constexpr uint8_t DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN = 22;
    constexpr uint8_t DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT = 23;
    constexpr uint8_t DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS = 24;
  } // namespace encoder

  namespace motor
  {
    constexpr uint8_t IS_SHAFT_DIRECTION_INVERTED = 30;
    constexpr uint8_t TCOOLTHRS = 31;
    constexpr uint8_t TPWMTHRS = 32;
    constexpr uint8_t MICROSTEPS = 33;
    constexpr uint8_t SEMIN = 34;
    constexpr uint8_t SEMAX = 35;
    constexpr uint8_t SEDN = 36;
  }

  namespace motor_monitor
  {
    constexpr uint8_t ACTIVE_SPEED_THRESHOLD = 40;
    constexpr uint8_t LOWER_POSITION_THRESHOLD = 41;
    constexpr uint8_t MAX_TIME_DIFF_BETWEEN_ENCODER_MEASUREMENTS_IN_MS = 42;
    constexpr uint8_t SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL = 43;
  } // namespace motor_monitor

  /********************************************************************************************************
   * Configs for the drawer
   *********************************************************************************************************/

  template <>
  struct ModuleSetting<drawer::MAX_SPEED>
  {
    using type = uint32_t;
    static constexpr type default_value = 35000;
  };

  template <>
  struct ModuleSetting<drawer::HOMING_SPEED>
  {
    using type = uint32_t;
    static constexpr type default_value = 1500;
  };

  template <>
  struct ModuleSetting<drawer::INITIAL_HOMING_SPEED>
  {
    using type = uint32_t;
    static constexpr type default_value = 1500;
  };

  template <>
  struct ModuleSetting<drawer::MOVING_IN_DECELERATION_DISTANCE>
  {
    using type = uint8_t;
    static constexpr type default_value = 30;
  };

  template <>
  struct ModuleSetting<drawer::MOVING_IN_FINAL_HOMING_DISTANCE>
  {
    using type = uint8_t;
    static constexpr type default_value = 1;
  };

  template <>
  struct ModuleSetting<drawer::MOVING_OUT_DECELERATION_DISTANCE>
  {
    using type = uint8_t;
    static constexpr type default_value = 70;
  };

  template <>
  struct ModuleSetting<drawer::MOVING_OUT_FINAL_SPEED>
  {
    using type = uint32_t;
    static constexpr type default_value = 5000;
  };

  template <>
  struct ModuleSetting<drawer::PUSH_IN_AUTO_CLOSE_SPEED>
  {
    using type = uint8_t;
    static constexpr type default_value = 120;
  };

  template <>
  struct ModuleSetting<drawer::PUSH_IN_AUTO_CLOSE_TMC_STALL_GUARD_VALUE>
  {
    using type = uint8_t;
    static constexpr type default_value = 75;
  };

  template <>
  struct ModuleSetting<drawer::PUSH_IN_WAIT_TIME_AFTER_STALL_GUARD_TRIGGERED_IN_MS>
  {
    using type = uint32_t;
    static constexpr type default_value = 200;
  };

  template <>
  struct ModuleSetting<drawer::STALL_GUARD_WAIT_TIME_AFTER_MOVEMENT_STARTED_IN_MS>
  {
    using type = uint32_t;
    static constexpr type default_value = 700;
  };

  template <>
  struct ModuleSetting<drawer::USE_TMC_STALL_GUARD>
  {
    using type = bool;
    static constexpr type default_value = false;
  };

  template <>
  struct ModuleSetting<drawer::USE_MOTOR_MONITOR_STALL_GUARD>
  {
    using type = bool;
    static constexpr type default_value = true;
  };

  template <>
  struct ModuleSetting<drawer::DRAWER_DEFAULT_ACCELERATION>
  {
    using type = uint8_t;
    static constexpr type default_value = 8;
  };

  /********************************************************************************************************
   * Configs for the encoder
   *********************************************************************************************************/

  template <>
  struct ModuleSetting<encoder::OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT>
  {
    using type = uint32_t;
    static constexpr type default_value = 85000;
  };

  template <>
  struct ModuleSetting<encoder::ENCODER_COUNT_DRAWER_MAX_EXTENT>
  {
    using type = uint32_t;
    static constexpr type default_value = 83000;
  };

  template <>
  struct ModuleSetting<encoder::DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN>
  {
    using type = uint32_t;
    static constexpr type default_value = 1000;
  };

  template <>
  struct ModuleSetting<encoder::DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT>
  {
    using type = float;
    static constexpr type default_value = 0.005;
  };

  template <>
  struct ModuleSetting<encoder::DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS>
  {
    using type = uint32_t;
    static constexpr type default_value = 500;
  };

  /********************************************************************************************************
   * Configs for the motor
   ********************************************************************************************************/

  template <>
  struct ModuleSetting<motor::IS_SHAFT_DIRECTION_INVERTED>
  {
    using type = bool;
    static constexpr type default_value = true;
  };
  template <>
  struct ModuleSetting<motor::TCOOLTHRS>
  {
    using type = uint32_t;
    static constexpr type default_value = 120;
  };

  template <>
  struct ModuleSetting<motor::TPWMTHRS>
  {
    using type = uint32_t;
    static constexpr type default_value = 10;
  };

  template <>
  struct ModuleSetting<motor::MICROSTEPS>
  {
    using type = uint16_t;
    static constexpr type default_value = 16;
  };

  template <>
  struct ModuleSetting<motor::SEMIN>
  {
    using type = uint8_t;
    static constexpr type default_value = 5;
  };

  template <>
  struct ModuleSetting<motor::SEMAX>
  {
    using type = uint8_t;
    static constexpr type default_value = 2;
  };

  template <>
  struct ModuleSetting<motor::SEDN>
  {
    using type = uint8_t;
    static constexpr type default_value = 1;
  };

  /********************************************************************************************************
   * Configs for the motor monitor
   ********************************************************************************************************/

  template <>
  struct ModuleSetting<motor_monitor::ACTIVE_SPEED_THRESHOLD>
  {
    using type = uint32_t;
    static constexpr type default_value = 800;
  };

  template <>
  struct ModuleSetting<motor_monitor::LOWER_POSITION_THRESHOLD>
  {
    using type = uint32_t;
    // Right after the drawer is homed, there can be a tension in the belt which can cause the motor to move a bit
    // We don't want to trigger the stall guard in this case, so define a threshold from which to start monitoring
    static constexpr type default_value = 200;
  };

  template <>
  struct ModuleSetting<motor_monitor::MAX_TIME_DIFF_BETWEEN_ENCODER_MEASUREMENTS_IN_MS>
  {
    using type = uint32_t;
    // usually one control loop takes 30-40ms (as of 29.07.2024)
    static constexpr type default_value = 100;
  };

  template <>
  struct ModuleSetting<motor_monitor::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL>
  {
    using type = float;
    static constexpr type default_value = 0.80;
  };

} // namespace module_config

#endif // MODULE_CONFIG__MODULE_CONFIG_DEFINES_HPP