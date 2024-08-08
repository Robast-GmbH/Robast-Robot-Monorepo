#include <catch2/catch_test_macros.hpp>
#include <utils/config_manager.hpp>

// How to run this:
// Go into the utils folder and run the following command:
//  cmake -S . -B build -DBUILD_TESTING=ON
// Go into the build folder and run the following command:
//   cmake --build .
// Run the test by executing this command:
//   ctest

TEST_CASE("Test if default configs are set correctly.", "[config_manager]")
{
  auto e_drawer_config = std::make_shared<drawer::ElectricalDrawerConfig>();
  auto encoder_config = std::make_shared<motor::EncoderConfig>();
  auto motor_config = std::make_shared<motor::MotorConfig>();
  auto motor_monitor_config = std::make_shared<motor::MotorMonitorConfig>();

  utils::ConfigManager config_manager(e_drawer_config, encoder_config, motor_config, motor_monitor_config);

  SECTION("Check if default configs are set correctly for the e-drawer config.")
  {
    REQUIRE(e_drawer_config->get_drawer_max_speed() ==
            module_config::ModuleSetting<module_config::drawer::MAX_SPEED>::default_value);
    REQUIRE(e_drawer_config->get_drawer_homing_speed() ==
            module_config::ModuleSetting<module_config::drawer::HOMING_SPEED>::default_value);
    REQUIRE(e_drawer_config->get_drawer_initial_homing_speed() ==
            module_config::ModuleSetting<module_config::drawer::INITIAL_HOMING_SPEED>::default_value);
    REQUIRE(e_drawer_config->get_drawer_moving_in_deceleration_distance() ==
            module_config::ModuleSetting<module_config::drawer::MOVING_IN_DECELERATION_DISTANCE>::default_value);
    REQUIRE(e_drawer_config->get_drawer_moving_in_final_homing_distance() ==
            module_config::ModuleSetting<module_config::drawer::MOVING_IN_FINAL_HOMING_DISTANCE>::default_value);
    REQUIRE(e_drawer_config->get_drawer_moving_out_deceleration_distance() ==
            module_config::ModuleSetting<module_config::drawer::MOVING_OUT_DECELERATION_DISTANCE>::default_value);
    REQUIRE(e_drawer_config->get_drawer_push_in_auto_close_speed() ==
            module_config::ModuleSetting<module_config::drawer::PUSH_IN_AUTO_CLOSE_SPEED>::default_value);
    REQUIRE(
      e_drawer_config->get_drawer_push_in_auto_close_stall_guard_value() ==
      module_config::ModuleSetting<module_config::drawer::PUSH_IN_AUTO_CLOSE_TMC_STALL_GUARD_VALUE>::default_value);
    REQUIRE(e_drawer_config->get_drawer_push_in_wait_time_after_stall_guard_triggered_in_ms() ==
            module_config::ModuleSetting<
              module_config::drawer::PUSH_IN_WAIT_TIME_AFTER_STALL_GUARD_TRIGGERED_IN_MS>::default_value);
    REQUIRE(e_drawer_config->get_drawer_stall_guard_wait_time_after_movement_started_in_ms() ==
            module_config::ModuleSetting<
              module_config::drawer::STALL_GUARD_WAIT_TIME_AFTER_MOVEMENT_STARTED_IN_MS>::default_value);
    REQUIRE(e_drawer_config->get_use_tmc_stall_guard() ==
            module_config::ModuleSetting<module_config::drawer::USE_TMC_STALL_GUARD>::default_value);
    REQUIRE(e_drawer_config->get_drawer_default_acceleration() ==
            module_config::ModuleSetting<module_config::drawer::DRAWER_DEFAULT_ACCELERATION>::default_value);
  }

  SECTION("Check if default configs are set correctly for the encoder config.")
  {
    REQUIRE(encoder_config->get_open_loop_count_drawer_max_extent() ==
            module_config::ModuleSetting<module_config::encoder::OPEN_LOOP_COUNT_DRAWER_MAX_EXTENT>::default_value);
    REQUIRE(encoder_config->get_encoder_count_drawer_max_extent() ==
            module_config::ModuleSetting<module_config::encoder::ENCODER_COUNT_DRAWER_MAX_EXTENT>::default_value);
    REQUIRE(
      encoder_config->get_drawer_position_open_loop_integral_gain() ==
      module_config::ModuleSetting<module_config::encoder::DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN>::default_value);
    REQUIRE(
      encoder_config->get_drawer_push_in_encoder_check_interval_ms() ==
      module_config::ModuleSetting<module_config::encoder::DRAWER_PUSH_IN_ENCODER_CHECK_INTERVAL_MS>::default_value);
    REQUIRE(encoder_config->get_drawer_push_in_threshold_in_percent_of_max_extent() ==
            module_config::ModuleSetting<
              module_config::encoder::DRAWER_PUSH_IN_THRESHOLD_IN_PERCENT_OF_MAX_EXTENT>::default_value);
  }

  SECTION("Check if default configs are set correctlx for the motor config.")
  {
    REQUIRE(motor_config->get_is_shaft_direction_inverted() ==
            module_config::ModuleSetting<module_config::motor::IS_SHAFT_DIRECTION_INVERTED>::default_value);
    REQUIRE(motor_config->get_tcoolthrs() ==
            module_config::ModuleSetting<module_config::motor::TCOOLTHRS>::default_value);
    REQUIRE(motor_config->get_tpwmthrs() ==
            module_config::ModuleSetting<module_config::motor::TPWMTHRS>::default_value);
    REQUIRE(motor_config->get_microsteps() ==
            module_config::ModuleSetting<module_config::motor::MICROSTEPS>::default_value);
    REQUIRE(motor_config->get_semin() == module_config::ModuleSetting<module_config::motor::SEMIN>::default_value);
    REQUIRE(motor_config->get_semax() == module_config::ModuleSetting<module_config::motor::SEMAX>::default_value);
    REQUIRE(motor_config->get_sedn() == module_config::ModuleSetting<module_config::motor::SEDN>::default_value);
  }

  SECTION("Check if default configs are set correctly for the motor monitor config.")
  {
    REQUIRE(motor_monitor_config->get_active_speed_threshold() ==
            module_config::ModuleSetting<module_config::motor_monitor::ACTIVE_SPEED_THRESHOLD>::default_value);
    REQUIRE(motor_monitor_config->get_lower_position_threshold() ==
            module_config::ModuleSetting<module_config::motor_monitor::LOWER_POSITION_THRESHOLD>::default_value);
    REQUIRE(motor_monitor_config->get_max_time_diff_between_encoder_measurements_in_ms() ==
            module_config::ModuleSetting<
              module_config::motor_monitor::MAX_TIME_DIFF_BETWEEN_ENCODER_MEASUREMENTS_IN_MS>::default_value);
    REQUIRE(motor_monitor_config->get_speed_deviation_in_percentage_for_stall() ==
            module_config::ModuleSetting<
              module_config::motor_monitor::SPEED_DEVIATION_IN_PERCENTAGE_FOR_STALL>::default_value);
  }
}