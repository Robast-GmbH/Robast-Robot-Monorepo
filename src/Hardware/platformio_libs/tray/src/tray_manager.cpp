#include "tray/tray_manager.hpp"

namespace tray
{
  TrayManager::TrayManager(const std::vector<TrayPinConfig>& tray_pin_configs,
                           const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
                           const std::shared_ptr<TwoWire> wire,
                           const std::shared_ptr<drawer::ElectricalDrawer> e_drawer,
                           const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config,
                           const std::shared_ptr<tray::TrayManagerConfig> tray_manager_config,
                           const float switch_pressed_threshold,
                           const float switch_new_reading_weight)
      : _onboard_led_driver{std::make_unique<led::OnboardLedDriver>(wire)},
        _e_drawer{e_drawer},
        _motor_monitor_config{motor_monitor_config},
        _tray_manager_config{tray_manager_config}
  {
    _electrical_tray_locks.reserve(tray_pin_configs.size());
    _tray_switches.reserve(tray_pin_configs.size());
    for (const auto& tray_pin_config : tray_pin_configs)
    {
      _electrical_tray_locks.push_back(std::make_unique<ElectricalTrayLock>(
        gpio_wrapper, tray_pin_config.power_open_pin_id, tray_pin_config.power_close_pin_id));
      _tray_switches.push_back(std::make_unique<switch_lib::Switch>(gpio_wrapper,
                                                                    tray_pin_config.sensor_lock_pin_id,
                                                                    switch_pressed_threshold,
                                                                    switch_lib::Switch::normally_closed,
                                                                    switch_new_reading_weight));
      _timestamp_last_tray_lock_opening.push_back(0);
      _reduced_speed_for_tray_lid.push_back(false);
    }
    for (uint8_t tray_id = 1; tray_id <= _electrical_tray_locks.size(); ++tray_id)
    {
      _tray_lid_positions.push_back(get_tray_lid_position(tray_id));
    }
  }

  uint8_t TrayManager::get_tray_lid_position(uint8_t tray_id)
  {
    const uint8_t POSITION_OFFSET = _tray_manager_config->get_position_offset_for_tray_lid_computation();
    const uint8_t tray_lid_position =
      ((255 - POSITION_OFFSET) * (_electrical_tray_locks.size() - tray_id)) / _electrical_tray_locks.size() +
      POSITION_OFFSET;
    return tray_lid_position;
  }

  void TrayManager::init(std::function<void()> set_enable_pin_high)
  {
    _onboard_led_driver->initialize(set_enable_pin_high);
  }

  void TrayManager::unlock_lock(uint8_t tray_id)
  {
    debug_printf("[TrayManager]: Unlocking lock for tray %d\n", tray_id);
    _electrical_tray_locks[tray_id - 1]->unlock();
    _timestamp_last_tray_lock_opening[tray_id - 1] = millis();
  }

  void TrayManager::set_tray_led_brightness(const uint8_t tray_id, const uint8_t led_row, const uint8_t brightness)
  {
    _onboard_led_driver->set_tray_led_brightness(tray_id, led_row, brightness);
  }

  void TrayManager::update_states()
  {
    update_tray_states();
  }

  void TrayManager::update_tray_states()
  {
    for (uint8_t tray_id = 1; tray_id <= _electrical_tray_locks.size(); ++tray_id)
    {
      _electrical_tray_locks[tray_id - 1]->update_state();
      // We have two problems with the switches at the moment:
      // 1. The switches are not reliable pressed when the tray is closed
      // 2. It takes a lot of resources to read the switches because they are connected to the port expander
      // _tray_switches[i]->update_sensor_value(); // TODO: Use that once switch work reliable
      handle_tray_just_opened(tray_id);

      handle_e_drawer_movement_to_close_lid(tray_id);
    }
  }

  void TrayManager::handle_tray_just_opened(uint8_t tray_id)
  {
    if (!_electrical_tray_locks[tray_id - 1]->is_tray_lock_opening_in_progress())
    {
      return;
    }

    // Usually we should close the lock once the switch isn't pressed anymore
    // which should indicate that the tray is open, but the switch is not pressed
    // reliably.
    // bool is_tray_open = !_tray_switches[vector_id]->is_switch_pressed(); // TODO: Use that once switch is reliable
    // Therefore we close the lock after some time which should ensure that the tray is open
    const uint32_t current_timestamp = millis();
    const uint32_t time_since_lock_was_opened = current_timestamp - _timestamp_last_tray_lock_opening[tray_id - 1];
    const bool has_lock_mechanism_time_passed =
      time_since_lock_was_opened >= _ELECTRICAL_TRAY_LOCK_MECHANISM_TIME_IN_MS;

    if (has_lock_mechanism_time_passed)
    {
      // this makes sure the lock automatically closes as soon as the drawer is opened
      _electrical_tray_locks[tray_id - 1]->set_expected_lock_state_current_step(lock::LockState::locked);

      debug_println("[TrayManager]: Triggered closing the lock because tray just openend!");
      _electrical_tray_locks[tray_id - 1]->set_is_tray_lock_opening_in_progress(false);
    }
  }

  void TrayManager::handle_e_drawer_movement_to_close_lid(uint8_t tray_id)
  {
    if (!_electrical_tray_locks[tray_id - 1]->is_drawer_opening_in_progress() || !_e_drawer->is_drawer_moving_in())
    {
      return;
    }

    const uint8_t distance_to_tray_lid = abs(_tray_lid_positions[tray_id - 1] - _e_drawer->get_current_position());
    const uint8_t threshold = _tray_manager_config->get_distance_to_tray_lid_threshold();

    if (!_reduced_speed_for_tray_lid[tray_id - 1] && distance_to_tray_lid < threshold)
    {
      _speed_deviation_in_percentage_for_stall_before_reduced_speed =
        _motor_monitor_config->get_speed_deviation_in_percentage_for_stall();
      _target_speed_before_reduced_speed = _e_drawer->get_target_speed();

      _e_drawer->set_target_speed_with_decelerating_ramp(_tray_manager_config->get_target_speed_to_close_tray_lid());
      _motor_monitor_config->set_speed_deviation_in_percentage_for_stall(
        _tray_manager_config->get_speed_deviation_in_percentage_for_stall_when_closing_lid());

      _reduced_speed_for_tray_lid[tray_id - 1] = true;

      debug_printf_green(
        "[TrayManager]: Reduced speed for tray lid %d because distance to tray lid is %u and threshold is %u and "
        "_speed_deviation_in_percentage_for_stall_before_reduced_speed is %f and _target_speed_before_reduced_speed is "
        "%u\n",
        tray_id,
        distance_to_tray_lid,
        threshold,
        _speed_deviation_in_percentage_for_stall_before_reduced_speed,
        _target_speed_before_reduced_speed);
    }
    else if (distance_to_tray_lid > threshold)
    {
      _motor_monitor_config->set_speed_deviation_in_percentage_for_stall(
        _speed_deviation_in_percentage_for_stall_before_reduced_speed);
      _e_drawer->set_target_speed_and_direction(_target_speed_before_reduced_speed, true);
      _reduced_speed_for_tray_lid[tray_id - 1] = false;
      _electrical_tray_locks[tray_id - 1]->set_is_drawer_opening_in_progress(false);
      debug_printf_green(
        "[TrayManager]: Restored speed for tray lid %d because distance to tray lid is %u and threshold is %u and "
        "_speed_deviation_in_percentage_for_stall_before_reduced_speed is %f and _target_speed_before_reduced_speed is "
        "%u\n",
        tray_id,
        distance_to_tray_lid,
        threshold,
        _speed_deviation_in_percentage_for_stall_before_reduced_speed,
        _target_speed_before_reduced_speed);
    }
  }
}   // namespace tray
