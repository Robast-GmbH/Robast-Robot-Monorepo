#include <catch2/catch_test_macros.hpp>
#include <utils/config_manager.hpp>

// How to run this:
// Go into the utils folder and run the following command:
//  cmake -S . -B build -DBUILD_TESTING=ON
// Go into the build folder and run the following command:
//   cmake --build .
// Run the test by executing this command:
//   ctest

TEST_CASE("Test if default configs are set correctly", "[config_manager]")
{
  auto e_drawer_config = std::make_shared<drawer_controller::ElectricalDrawerConfig>();
  auto encoder_config = std::make_shared<drawer_controller::EncoderConfig>();
  auto motor_config = std::make_shared<drawer_controller::MotorConfig>();
  auto motor_monitor_config = std::make_shared<drawer_controller::MotorMonitorConfig>();

  drawer_controller::ConfigManager config_manager(e_drawer_config, encoder_config, motor_config, motor_monitor_config);

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
  REQUIRE(e_drawer_config->get_drawer_push_in_auto_close_stall_guard_value() ==
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