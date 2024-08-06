#include "utils/config_manager.hpp"

namespace drawer_controller
{
  ConfigManager::ConfigManager(const std::shared_ptr<ElectricalDrawerConfig> drawer_config,
                               const std::shared_ptr<EncoderConfig> encoder_config,
                               const std::shared_ptr<MotorConfig> motor_config,
                               const std::shared_ptr<MotorMonitorConfig> motor_monitor_config)
      : _drawer_config{drawer_config},
        _encoder_config{encoder_config},
        _motor_config{motor_config},
        _motor_monitor_config{motor_monitor_config}
  {
    set_default_configs();
  }

  bool ConfigManager::set_config(const uint8_t config_id, const uint32_t config_value)
  {
    switch (config_id)
    {
      case module_config::drawer::MAX_SPEED:
        _drawer_config->set_drawer_max_speed(
          std::bit_cast<module_config::ModuleSetting<module_config::drawer::MAX_SPEED>::type>(config_value));
        break;

      case module_config::drawer::HOMING_SPEED:
        _drawer_config->set_drawer_homing_speed(
          std::bit_cast<module_config::ModuleSetting<module_config::drawer::HOMING_SPEED>::type>(config_value));
        break;

      case module_config::drawer::INITIAL_HOMING_SPEED:
        _drawer_config->set_drawer_initial_homing_speed(
          std::bit_cast<module_config::ModuleSetting<module_config::drawer::INITIAL_HOMING_SPEED>::type>(config_value));
        break;

      case module_config::drawer::MOVING_IN_DECELERATION_DISTANCE:
        _drawer_config->set_drawer_moving_in_deceleration_distance(
          static_cast<module_config::ModuleSetting<module_config::drawer::MOVING_IN_DECELERATION_DISTANCE>::type>(
            config_value));
        break;

      case module_config::drawer::MOVING_IN_FINAL_HOMING_DISTANCE:
        _drawer_config->set_drawer_moving_in_final_homing_distance(
          static_cast<module_config::ModuleSetting<module_config::drawer::MOVING_IN_FINAL_HOMING_DISTANCE>::type>(
            config_value));
        break;

      case module_config::drawer::MOVING_OUT_DECELERATION_DISTANCE:
        _drawer_config->set_drawer_moving_out_deceleration_distance(
          static_cast<module_config::ModuleSetting<module_config::drawer::MOVING_OUT_DECELERATION_DISTANCE>::type>(
            config_value));
        break;

      case module_config::drawer::PUSH_IN_AUTO_CLOSE_SPEED:
        _drawer_config->set_drawer_push_in_auto_close_speed(
          static_cast<module_config::ModuleSetting<module_config::drawer::PUSH_IN_AUTO_CLOSE_SPEED>::type>(
            config_value));
        break;

      case module_config::drawer::PUSH_IN_AUTO_CLOSE_TMC_STALL_GUARD_VALUE:
        _drawer_config->set_drawer_push_in_auto_close_stall_guard_value(
          static_cast<
            module_config::ModuleSetting<module_config::drawer::PUSH_IN_AUTO_CLOSE_TMC_STALL_GUARD_VALUE>::type>(
            config_value));
        break;

      case module_config::drawer::PUSH_IN_WAIT_TIME_AFTER_STALL_GUARD_TRIGGERED_IN_MS:
        _drawer_config->set_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms(
          std::bit_cast<module_config::ModuleSetting<
            module_config::drawer::PUSH_IN_WAIT_TIME_AFTER_STALL_GUARD_TRIGGERED_IN_MS>::type>(config_value));
        break;

      case module_config::drawer::STALL_GUARD_WAIT_TIME_AFTER_MOVEMENT_STARTED_IN_MS:
        _drawer_config->set_drawer_stall_guard_wait_time_after_movement_started_in_ms(
          std::bit_cast<module_config::ModuleSetting<
            module_config::drawer::STALL_GUARD_WAIT_TIME_AFTER_MOVEMENT_STARTED_IN_MS>::type>(config_value));
        break;

      case module_config::drawer::USE_TMC_STALL_GUARD:
        _drawer_config->set_use_tmc_stall_guard(
          static_cast<module_config::ModuleSetting<module_config::drawer::USE_TMC_STALL_GUARD>::type>(config_value));
        break;

      case module_config::drawer::DRAWER_DEFAULT_ACCELERATION:
        _drawer_config->set_drawer_default_acceleration(
          static_cast<module_config::ModuleSetting<module_config::drawer::DRAWER_DEFAULT_ACCELERATION>::type>(
            config_value));
        break;

      case module_config::encoder::OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT:
        _encoder_config->set_open_loop_count_drawer_max_extent(
          std::bit_cast<module_config::ModuleSetting<module_config::encoder::OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT>::type>(
            config_value));
        break;

      case module_config::encoder::ENCODER_COUNT_DRAWER_MAX_EXTENT:
        _encoder_config->set_encoder_count_drawer_max_extent(
          std::bit_cast<module_config::ModuleSetting<module_config::encoder::ENCODER_COUNT_DRAWER_MAX_EXTENT>::type>(
            config_value));
        break;

      case module_config::encoder::DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN:
        _encoder_config->set_drawer_position_open_loop_integral_gain(
          std::bit_cast<
            module_config::ModuleSetting<module_config::encoder::DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN>::type>(
            config_value));
        break;

      case module_config::encoder::DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT:
        _encoder_config->set_drawer_push_in_threshold_in_percent_of_max_extent(
          std::bit_cast<module_config::ModuleSetting<
            module_config::encoder::DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT>::type>(config_value));

        break;

      case module_config::encoder::DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS:
        _encoder_config->set_drawer_push_in_encoder_check_interval_ms(
          std::bit_cast<
            module_config::ModuleSetting<module_config::encoder::DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS>::type>(
            config_value));
        break;

      case module_config::motor::IS_SHAFT_DIRECTION_INVERTED:
        _motor_config->set_is_shaft_direction_inverted(
          static_cast<module_config::ModuleSetting<module_config::motor::IS_SHAFT_DIRECTION_INVERTED>::type>(
            config_value));
        break;

      case module_config::motor_monitor::ACTIVE_SPEED_THRESHOLD:
        _motor_monitor_config->set_active_speed_threshold(
          std::bit_cast<module_config::ModuleSetting<module_config::motor_monitor::ACTIVE_SPEED_THRESHOLD>::type>(
            config_value));
        break;

      case module_config::motor_monitor::LOWER_POSITION_THRESHOLD:
        _motor_monitor_config->set_lower_position_threshold(
          std::bit_cast<module_config::ModuleSetting<module_config::motor_monitor::LOWER_POSITION_THRESHOLD>::type>(
            config_value));
        break;

      case module_config::motor_monitor::MAX_TIME_DIFF_BETWEEN_ENCODER_MEASUREMENTS_IN_MS:
        _motor_monitor_config->set_max_time_diff_between_encoder_measurements_in_ms(
          std::bit_cast<module_config::ModuleSetting<
            module_config::motor_monitor::MAX_TIME_DIFF_BETWEEN_ENCODER_MEASUREMENTS_IN_MS>::type>(config_value));
        break;

      case module_config::motor_monitor::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL:
        _motor_monitor_config->set_speed_deviation_in_percentage_for_stall(
          std::bit_cast<
            module_config::ModuleSetting<module_config::motor_monitor::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL>::type>(
            config_value));
        break;

      default:
        return false;
    }
    return true;
  }

  void ConfigManager::set_default_configs()
  {
    set_default_drawer_config();

    set_default_encoder_config();

    set_motor_config();

    set_default_motor_monitor_config();
  }

  void ConfigManager::set_default_drawer_config()
  {
    set_config(module_config::drawer::MAX_SPEED,
               std::bit_cast<uint32_t>(module_config::ModuleSetting<module_config::drawer::MAX_SPEED>::default_value));

    set_config(
      module_config::drawer::HOMING_SPEED,
      std::bit_cast<uint32_t>(module_config::ModuleSetting<module_config::drawer::HOMING_SPEED>::default_value));

    set_config(module_config::drawer::INITIAL_HOMING_SPEED,
               std::bit_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::drawer::INITIAL_HOMING_SPEED>::default_value));

    set_config(module_config::drawer::MOVING_IN_DECELERATION_DISTANCE,
               static_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::drawer::MOVING_IN_DECELERATION_DISTANCE>::default_value));

    set_config(module_config::drawer::MOVING_IN_FINAL_HOMING_DISTANCE,
               static_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::drawer::MOVING_IN_FINAL_HOMING_DISTANCE>::default_value));

    set_config(module_config::drawer::MOVING_OUT_DECELERATION_DISTANCE,
               static_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::drawer::MOVING_OUT_DECELERATION_DISTANCE>::default_value));

    set_config(module_config::drawer::PUSH_IN_AUTO_CLOSE_SPEED,
               static_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::drawer::PUSH_IN_AUTO_CLOSE_SPEED>::default_value));

    set_config(
      module_config::drawer::PUSH_IN_AUTO_CLOSE_TMC_STALL_GUARD_VALUE,
      static_cast<uint32_t>(
        module_config::ModuleSetting<module_config::drawer::PUSH_IN_AUTO_CLOSE_TMC_STALL_GUARD_VALUE>::default_value));

    set_config(module_config::drawer::PUSH_IN_WAIT_TIME_AFTER_STALL_GUARD_TRIGGERED_IN_MS,
               std::bit_cast<uint32_t>(
                 module_config::ModuleSetting<
                   module_config::drawer::PUSH_IN_WAIT_TIME_AFTER_STALL_GUARD_TRIGGERED_IN_MS>::default_value));

    set_config(module_config::drawer::STALL_GUARD_WAIT_TIME_AFTER_MOVEMENT_STARTED_IN_MS,
               std::bit_cast<uint32_t>(
                 module_config::ModuleSetting<
                   module_config::drawer::STALL_GUARD_WAIT_TIME_AFTER_MOVEMENT_STARTED_IN_MS>::default_value));

    set_config(
      module_config::drawer::USE_TMC_STALL_GUARD,
      static_cast<uint32_t>(module_config::ModuleSetting<module_config::drawer::USE_TMC_STALL_GUARD>::default_value));

    set_config(module_config::drawer::DRAWER_DEFAULT_ACCELERATION,
               static_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::drawer::DRAWER_DEFAULT_ACCELERATION>::default_value));
  }

  void ConfigManager::set_default_encoder_config()
  {
    set_config(
      module_config::encoder::OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT,
      std::bit_cast<uint32_t>(
        module_config::ModuleSetting<module_config::encoder::OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT>::default_value));

    set_config(module_config::encoder::ENCODER_COUNT_DRAWER_MAX_EXTENT,
               std::bit_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::encoder::ENCODER_COUNT_DRAWER_MAX_EXTENT>::default_value));

    set_config(
      module_config::encoder::DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN,
      std::bit_cast<uint32_t>(
        module_config::ModuleSetting<module_config::encoder::DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN>::default_value));

    set_config(module_config::encoder::DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT,
               std::bit_cast<uint32_t>(
                 module_config::ModuleSetting<
                   module_config::encoder::DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT>::default_value));

    set_config(
      module_config::encoder::DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS,
      std::bit_cast<uint32_t>(
        module_config::ModuleSetting<module_config::encoder::DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS>::default_value));
  }

  void ConfigManager::set_motor_config()
  {
    set_config(module_config::motor::IS_SHAFT_DIRECTION_INVERTED,
               static_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::motor::IS_SHAFT_DIRECTION_INVERTED>::default_value));
  }

  void ConfigManager::set_default_motor_monitor_config()
  {
    set_config(module_config::motor_monitor::ACTIVE_SPEED_THRESHOLD,
               std::bit_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::motor_monitor::ACTIVE_SPEED_THRESHOLD>::default_value));

    set_config(module_config::motor_monitor::LOWER_POSITION_THRESHOLD,
               std::bit_cast<uint32_t>(
                 module_config::ModuleSetting<module_config::motor_monitor::LOWER_POSITION_THRESHOLD>::default_value));

    set_config(module_config::motor_monitor::MAX_TIME_DIFF_BETWEEN_ENCODER_MEASUREMENTS_IN_MS,
               std::bit_cast<uint32_t>(
                 module_config::ModuleSetting<
                   module_config::motor_monitor::MAX_TIME_DIFF_BETWEEN_ENCODER_MEASUREMENTS_IN_MS>::default_value));

    set_config(
      module_config::motor_monitor::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL,
      std::bit_cast<uint32_t>(module_config::ModuleSetting<
                              module_config::motor_monitor::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL>::default_value));
  }

}   // namespace drawer_controller