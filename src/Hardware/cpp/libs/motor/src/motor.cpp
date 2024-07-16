#include "motor/motor.hpp"

namespace stepper_motor
{
  bool Motor::_is_stalled = false;

  Motor::Motor(const uint8_t driver_address,
               const std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper,
               const StepperPinIdConfig &stepper_pin_id_config,
               const bool shaft_direction_is_inverted)
      : _gpio_wrapper{gpio_wrapper},
        _driver{std::make_unique<TMC2209Stepper>(&SERIAL_PORT, R_SENSE, driver_address)},
        _stepper_enn_tmc2209_pin_id{stepper_pin_id_config.stepper_enn_tmc2209_pin_id},
        _stepper_stdby_tmc2209_pin_id{stepper_pin_id_config.stepper_stdby_tmc2209_pin_id},
        _stepper_spread_pin_id{stepper_pin_id_config.stepper_spread_pin_id},
        _stepper_dir_pin_id{stepper_pin_id_config.stepper_dir_pin_id},
        _stepper_diag_pin_id{stepper_pin_id_config.stepper_diag_pin_id},
        _stepper_index_pin_id{stepper_pin_id_config.stepper_index_pin_id},
        _stepper_step_pin_id{stepper_pin_id_config.stepper_step_pin_id},
        _shaft_direction_is_inverted{shaft_direction_is_inverted}
  {
    _gpio_wrapper->set_pin_mode(_stepper_enn_tmc2209_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());
    _gpio_wrapper->set_pin_mode(_stepper_stdby_tmc2209_pin_id, _gpio_wrapper->get_gpio_output_pin_mode());
  }

  void Motor::init()
  {
    debug_println("Starting Motor init!");

    _gpio_wrapper->digital_write(_stepper_enn_tmc2209_pin_id, LOW);
    _gpio_wrapper->digital_write(_stepper_stdby_tmc2209_pin_id, LOW);

    // After enabling the stepper driver, give it a small amount of time to get running before sending commands
    delay(100);

    SERIAL_PORT.begin(115200);

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
    _driver->VACTUAL(_active_speed);

    // Comparator blank time. This time needs to safely cover the switching
    // event and the duration of the ringing on the sense resistor. For most
    // applications, a setting of 16 or 24 is good. For highly capacitive
    // loads, a setting of 32 or 40 will be required.
    _driver->blank_time(24);

    _driver->rms_current(670);   // mA
    _driver->microsteps(16);

    // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
    _driver->TCOOLTHRS(0xFFFFF);   // 20bit max

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
    _driver->SGTHRS(STALL_VALUE);

    _driver->shaft(direction_to_shaft_bool(Direction::clockwise));

    debug_println("\nTesting connection...");
    uint8_t result = _driver->test_connection();

    if (result)
    {
      debug_println("failed!");
      debug_println("Likely cause: ");

      switch (result)
      {
        case 1:
          debug_println("loose connection");
          break;
        case 2:
          debug_println("no power");
          break;
      }

      debug_println("Fix the problem and reset board.");

      // We need this delay or messages above don't get fully printed out
      delay(100);
      abort();
    }

    // set_stall_guard(true);

    debug_println("Stepper initialized");
  }

  void Motor::stall_ISR()
  {
    _is_stalled = true;
  }

  bool Motor::get_is_stalled() const
  {
    return _is_stalled;
  }

  // TODO@Jacob: implement Stall guard with ticket: https://robast.atlassian.net/browse/RE-1446
  //  void Motor::set_stall_guard(bool enable)
  //  {
  //    if (_is_stall_guard_enabled && !enable)
  //    {
  //      detachInterrupt(STEPPER_DIAG_PIN);   // TODO@Jacob: Use interrup from port expander
  //      _is_stall_guard_enabled = false;
  //    }
  //    else if (enable)
  //    {
  //      attachInterrupt(STEPPER_DIAG_PIN, Motor::stall_ISR, RISING);   // TODO@Jacob: Use interrup from port expander
  //    }
  //  }

  // void Motor::reset_stall_guard()
  // {
  //   setSpeed(0, 0);
  //   _driver->VACTUAL(_speed);
  //   detachInterrupt(STEPPER_DIAG_PIN);   // TODO@Jacob: Use interrup from port expander
  //   _gpio_wrapper->digital_write(_stepper_enn_tmc2209_pin_id, HIGH);
  //   delay(100);
  //   _gpio_wrapper->digital_write(_stepper_enn_tmc2209_pin_id, LOW);
  //   delay(500);
  //   attachInterrupt(STEPPER_DIAG_PIN, stall_ISR, RISING);
  //   _is_stalled = false;
  // }

  void Motor::handle_motor_control(int32_t current_position_int32)
  {
    if (!_speed_ramp_in_progress)
    {
      return;
    }

    if (get_active_speed() < get_target_speed())
    {
      handle_time_dependent_acceleration();
    }
    if (get_active_speed() > get_target_speed())
    {
      handle_position_dependent_deceleration(current_position_int32);
    }
  }

  void Motor::handle_time_dependent_acceleration()
  {
    uint32_t delta_speed_uint32 = get_dt_since_start_in_ms() * _acceleration;

    uint32_t new_active_speed_uint32 = _starting_speed_before_ramp_uint32 + delta_speed_uint32;

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
    int32_t new_active_speed_int32 = calculate_new_active_speed(current_position_int32);
    _speed_ramp_in_progress = !(new_active_speed_int32 < get_target_speed());
    new_active_speed_int32 = _speed_ramp_in_progress ? new_active_speed_int32 : get_target_speed();
    set_active_speed(new_active_speed_int32);
  }

  int32_t Motor::calculate_new_active_speed(int32_t current_position_int32) const
  {
    uint32_t distance_travelled_uint32 = abs(current_position_int32 - _starting_position_int32);

    uint32_t total_delta_speed_uint32 =
      abs((int32_t) get_target_speed() - (int32_t) _starting_speed_before_ramp_uint32);

    uint32_t delta_speed_uint32 = (total_delta_speed_uint32 * distance_travelled_uint32) / _ramp_distance_uint32;

    int32_t new_active_speed_int32 = (int32_t) _starting_speed_before_ramp_uint32 - (int32_t) delta_speed_uint32;

    return new_active_speed_int32;
  }

  uint32_t Motor::get_dt_since_start_in_ms() const
  {
    uint32_t current_timestemp = millis();
    return current_timestemp - _start_ramp_timestamp;
  }

  void Motor::finish_speed_ramp(uint32_t final_speed)
  {
    debug_println("The speed ramp-up/ramp-down is finished, setting the motor speed to the target speed directly!");
    set_active_speed(final_speed);
    _speed_ramp_in_progress = false;
  }

  void Motor::set_active_speed(uint32_t active_speed)
  {
    _active_speed = active_speed;

    _driver->VACTUAL(_active_speed);
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
    if (acceleration == INSTANT_ACCELERATION)
    {
      set_active_speed(target_speed);
      _target_speed = target_speed;
      return;
    }

    if (target_speed != _target_speed)
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
    debug_print("Status: ");
    debug_print_with_base(_driver->SG_RESULT(), DEC);
    debug_print(" ");
    debug_print_with_base(_driver->SG_RESULT() < STALL_VALUE, DEC);
    debug_print(" ");
    byte drawer_diag_state;
    _gpio_wrapper->digital_read(_stepper_diag_pin_id, drawer_diag_state);
    debug_print(drawer_diag_state);
    debug_print(" ");
    debug_println_with_base(_driver->cs2rms(_driver->cs_actual()), DEC);
  }

}   // namespace stepper_motor