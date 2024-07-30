#include "motor/motor.hpp"

namespace stepper_motor
{
  Motor::Motor(const uint8_t driver_address,
               const std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
               const StepperPinIdConfig &stepper_pin_id_config,
               const bool shaft_direction_is_inverted)
      : _gpio_wrapper{gpio_wrapper},
        _driver{std::make_unique<TMC2209Stepper>(&SERIAL_PORT, R_SENSE, driver_address)},
        _driver_is_enabled{false},
        _stepper_enn_tmc2209_pin_id{stepper_pin_id_config.stepper_enn_tmc2209_pin_id},
        _stepper_stdby_tmc2209_pin_id{stepper_pin_id_config.stepper_stdby_tmc2209_pin_id},
        _stepper_spread_pin_id{stepper_pin_id_config.stepper_spread_pin_id},
        _stepper_dir_pin_id{stepper_pin_id_config.stepper_dir_pin_id},
        _stepper_diag_pin_id{stepper_pin_id_config.stepper_diag_pin_id},
        _stepper_index_pin_id{stepper_pin_id_config.stepper_index_pin_id},
        _stepper_step_pin_id{stepper_pin_id_config.stepper_step_pin_id},
        _port_expander_ninterrupt_pin_id{stepper_pin_id_config.port_expander_ninterrupt_pin_id},
        _shaft_direction_is_inverted{shaft_direction_is_inverted},
        _is_stalled{false},
        _stall_guard_reader{std::make_unique<drawer_controller::Switch>(gpio_wrapper,
                                                                        stepper_pin_id_config.stepper_diag_pin_id,
                                                                        STALL_GUARD_READER_THRESHOLD,
                                                                        drawer_controller::Switch::normally_open,
                                                                        STALL_GUARD_READER_WEIGHT_NEW_READINGS)},
        _current_stall_guard_value{STALL_DEFAULT_VALUE}
  {
    _gpio_wrapper->set_pin_mode(_stepper_enn_tmc2209_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());
    _gpio_wrapper->set_pin_mode(_stepper_stdby_tmc2209_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());
  }

  void Motor::init()
  {
    debug_println("[Motor]: Starting Motor init!");

    _gpio_wrapper->digital_write(_stepper_stdby_tmc2209_pin_id, LOW);

    SERIAL_PORT.begin(115200);

    setup_driver();

    debug_println("[Motor]: Testing connection...");
    uint8_t result = _driver->test_connection();

    if (result)
    {
      debug_println("[Motor]: failed!");
      debug_println("[Motor]: Likely cause: ");

      switch (result)
      {
        case 1:
          debug_println("[Motor]: loose connection");
          break;
        case 2:
          debug_println("[Motor]: no power");
          break;
      }

      debug_println("[Motor]: Fix the problem and reset board.");

      // We need this delay or messages above don't get fully printed out
      delay(100);
      abort();
    }

    debug_println("[Motor]: Stepper initialized. Disabling driver.");

    disable_driver();
  }

  void Motor::setup_driver()
  {
    if (_driver_is_enabled)
    {
      return;
    }

    enable_driver();

    // After enabling the stepper driver, give it a small amount of time to get running before sending commands
    delay(100);

    _driver->begin();

    // Sets the slow decay time (off time) [1... 15]. This setting also limits
    // the maximum chopper frequency. For operation with StealthChop,
    // this parameter is not used, but it is required to enable the motor.
    // In case of operation with StealthChop only, any setting is OK.
    _driver->toff(TOFF_VALUE);

    // VACTUAL allows moving the motor by UART control.
    // It gives the motor velocity in +-(2^23)-1 [μsteps / t]
    // 0: Normal operation. Driver reacts to STEP input.
    // /=0: Motor moves with the velocity given by VACTUAL.
    // Step pulses can be monitored via INDEX output.
    // The motor direction is controlled by the sign of VACTUAL.
    _driver->VACTUAL(0);

    // Comparator blank time. This time needs to safely cover the switching
    // event and the duration of the ringing on the sense resistor. For most
    // applications, a setting of 16 or 24 is good. For highly capacitive
    // loads, a setting of 32 or 40 will be required.
    _driver->blank_time(24);

    _driver->rms_current(670);   // mA
    _driver->microsteps(16);

    // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
    // Mind that TCOOLTHRS is compared with TSTEP and TSTEP is scaling INVERSELY with the velocity.
    // So the higher the velocity, the lower the TSTEP and the higher the TCOOLTHRS has to be.
    // TCOOLTHRS >= TSTEP >= TPWMTHRS
    _driver->TCOOLTHRS(TCOOLTHRS_VALUE);   // 20bit max

    // Upper threshold velocity for switching off smart energy CoolStep and StallGuard to DIAG output
    _driver->TPWMTHRS(TPWMTHRS_VALUE);

    // CoolStep lower threshold [0... 15].
    // If SG_RESULT goes below this threshold, CoolStep increases the current to both coils.
    // 0: disable CoolStep
    _driver->semin(5);

    // CoolStep upper threshold [0... 15].
    // If SG is sampled equal to or above this threshold enough times,
    // CoolStep decreases the current to both coils.
    _driver->semax(2);

    // Sets the number of StallGuard2 readings above the upper threshold necessary
    // for each current decrement of the motor current.
    _driver->sedn(0b01);

    // StallGuard4 threshold [0... 255] level for stall detection. It compensates for
    // motor specific characteristics and controls sensitivity. A higher value gives a higher
    // sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
    // indicate a stall. The double of this value is compared to SG_RESULT.
    // The stall output becomes active if SG_RESULT fall below this value.
    _driver->SGTHRS(_current_stall_guard_value);
  }

  void Motor::enable_driver()
  {
    if (_driver_is_enabled)
    {
      return;
    }
    debug_println("[Motor]: Enable motor driver!");
    _gpio_wrapper->digital_write(_stepper_enn_tmc2209_pin_id, LOW);
    _driver_is_enabled = true;
  }

  void Motor::disable_driver()
  {
    if (!_driver_is_enabled)
    {
      return;
    }
    debug_println("[Motor]: Disabling motor driver!");
    _gpio_wrapper->digital_write(_stepper_enn_tmc2209_pin_id, HIGH);
    _driver_is_enabled = false;
  }

  bool Motor::get_is_stalled() const
  {
    return _is_stalled;
  }

  void Motor::set_stall_guard(uint8_t stall_guard_value)
  {
    debug_printf("[Motor]: Setting stall guard value to: %d\n", stall_guard_value);
    _current_stall_guard_value = stall_guard_value;
    if (_driver_is_enabled)
    {
      _driver->SGTHRS(_current_stall_guard_value);
    }
  }

  void Motor::reset_stall_guard()
  {
    debug_println("[Motor]: Resetting stall guard!");
    _is_stalled = false;
    disable_driver();
  }

  void Motor::handle_motor_control(int32_t current_position_int32)
  {
    read_stall_guard_pin();

    if (!_speed_ramp_in_progress)
    {
      return;
    }

    // print active speed and target speed
    debug_printf(
      "[Motor, handle_motor_control]: Active speed: %u, Target speed: %u\n", get_active_speed(), get_target_speed());

    (get_active_speed() < get_target_speed()) ? handle_time_dependent_acceleration()
                                              : handle_position_dependent_deceleration(current_position_int32);
  }

  void Motor::read_stall_guard_pin()
  {
    _stall_guard_reader->update_sensor_value();

    _is_stalled = _stall_guard_reader->is_switch_pressed();

    if (_is_stalled)
    {
      debug_printf("[Motor]: Stall detected for TSTEP %u\n", _driver->TSTEP());
    }
  }

  void Motor::handle_time_dependent_acceleration()
  {
    uint32_t delta_speed_uint32 = get_dt_since_start_in_ms() * _acceleration;

    uint32_t new_active_speed_uint32 = _starting_speed_before_ramp_uint32 + delta_speed_uint32;

    debug_printf("[Motor, handle_time_dependent_acceleration]: New active speed: %d\n", new_active_speed_uint32);

    if (new_active_speed_uint32 > get_target_speed())
    {
      finish_speed_ramp(get_target_speed());
    }
    else
    {
      set_active_speed(new_active_speed_uint32);
    }
  }

  void Motor::handle_position_dependent_deceleration(int32_t current_position_int32)
  {
    uint32_t new_active_speed_uint32 = calculate_new_active_speed(current_position_int32);
    _speed_ramp_in_progress = !(new_active_speed_uint32 < get_target_speed());
    new_active_speed_uint32 = _speed_ramp_in_progress ? new_active_speed_uint32 : get_target_speed();
    debug_printf("[Motor, handle_position_dependent_deceleration]: New active speed: %d\n", new_active_speed_uint32);
    set_active_speed(new_active_speed_uint32);
  }

  uint32_t Motor::calculate_new_active_speed(int32_t current_position_int32) const
  {
    uint32_t distance_travelled_uint32 = abs(current_position_int32 - _starting_position_int32);

    uint32_t total_delta_speed_uint32 =
      abs(static_cast<int32_t>(get_target_speed()) - static_cast<int32_t>(_starting_speed_before_ramp_uint32));

    uint32_t delta_speed_uint32 = (total_delta_speed_uint32 * distance_travelled_uint32) / _ramp_distance_uint32;

    int32_t new_active_speed_int32 = (int32_t) _starting_speed_before_ramp_uint32 - (int32_t) delta_speed_uint32;

    if (new_active_speed_int32 < 0)
    {
      return 0;
    }
    else
    {
      return static_cast<uint32_t>(new_active_speed_int32);
    }
  }

  uint32_t Motor::get_dt_since_start_in_ms() const
  {
    uint32_t current_timestemp = millis();
    return current_timestemp - _start_ramp_timestamp;
  }

  void Motor::finish_speed_ramp(uint32_t final_speed)
  {
    debug_println(
      "[Motor]: The speed ramp-up/ramp-down is finished, setting the motor speed to the target speed directly!");
    set_active_speed(final_speed);
    _speed_ramp_in_progress = false;
  }

  void Motor::set_active_speed(uint32_t active_speed)
  {
    debug_printf("[Motor]: Setting active speed to %u\n", active_speed);

    _active_speed = active_speed;

    setup_driver();

    _driver->VACTUAL(_active_speed);

    if (_active_speed == 0)
    {
      disable_driver();
    }
  }

  uint32_t Motor::get_active_speed() const
  {
    return _active_speed;
  }

  void Motor::set_target_speed_with_decelerating_ramp(uint32_t target_speed,
                                                      int32_t ramp_distance_int32,
                                                      int32_t starting_position_int32)
  {
    if (target_speed != _target_speed)
    {
      _target_speed = target_speed;
      _starting_speed_before_ramp_uint32 = _active_speed;
      _ramp_distance_uint32 = ramp_distance_int32;
      _starting_position_int32 = starting_position_int32;
      _speed_ramp_in_progress = true;
    }
  }

  void Motor::set_target_speed_with_accelerating_ramp(uint32_t target_speed, int16_t acceleration)
  {
    if (acceleration == INSTANT_ACCELERATION || target_speed <= _target_speed)
    {
      debug_printf("[Motor]: Setting target speed with INSTANT_ACCELERATION to %u\n", target_speed);
      set_active_speed(target_speed);
      _target_speed = target_speed;
      return;
    }

    if (target_speed > _target_speed)
    {
      _target_speed = target_speed;
      _starting_speed_before_ramp_uint32 = _active_speed;
      _acceleration = acceleration;
      _start_ramp_timestamp = millis();   // TODO@Jacob: How to handle overflow?
      _speed_ramp_in_progress = true;
    }
  }

  void Motor::set_target_speed_instantly(uint32_t target_speed)
  {
    debug_printf("[Motor]: Setting target speed instantly to %u\n", target_speed);
    set_active_speed(target_speed);
    _target_speed = target_speed;
  }

  uint32_t Motor::get_target_speed() const
  {
    return _target_speed;
  }

  bool Motor::direction_to_shaft_bool(Direction direction)
  {
    if (_shaft_direction_is_inverted)
    {
      return direction == Direction::counter_clockwise;
    }
    else
    {
      return direction == Direction::clockwise;
    }
  }

  void Motor::set_direction(Direction direction)
  {
    // When the direction is changing, the stall guard could be accidentally triggered therefore detach interrupt
    // detachInterrupt(STEPPER_DIAG_PIN);
    _shaft_direction = direction;
    _driver->shaft(direction_to_shaft_bool(direction));
    // delay(500);
    // attachInterrupt(STEPPER_DIAG_PIN, stall_ISR, RISING);
  }

  Direction Motor::get_direction() const
  {
    return _shaft_direction;
  }

  void Motor::print_status()
  {
    debug_print("[Motor]: Status: ");
    debug_print_with_base(_driver->SG_RESULT(), DEC);
    debug_print(" ");
    debug_print_with_base(_driver->SG_RESULT() < STALL_DEFAULT_VALUE, DEC);
    debug_print(" ");
    byte drawer_diag_state;
    _gpio_wrapper->digital_read(_stepper_diag_pin_id, drawer_diag_state);
    debug_print(drawer_diag_state);
    debug_print(" ");
    debug_println_with_base(_driver->cs2rms(_driver->cs_actual()), DEC);
  }

}   // namespace stepper_motor