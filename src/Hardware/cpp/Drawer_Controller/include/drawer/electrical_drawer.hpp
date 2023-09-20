#ifndef DRAWER_CONTROLLER_ELECTRICAL_DRAWER_HPP
#define DRAWER_CONTROLLER_ELECTRICAL_DRAWER_HPP

#include <Arduino.h>

#include <memory>

#include "can/can_db.hpp"
#include "can/can_utils.hpp"
#include "interfaces/i_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "peripherals/electrical_lock.hpp"
#include "peripherals/encoder.hpp"
#include "peripherals/motor.hpp"

#define DRAWER_MAX_SPEED    35000
#define DRAWER_HOMING_SPEED 300

// The drawer starts to decelerate in dependency of the traveled distance
#define DRAWER_MOVING_IN_DECELERATION_DISTANCE  50   // distance to the target position to start deceleration (max 255)
#define DRAWER_MOVING_IN_FINAL_HOMING_DISTANCE  1    // the end of the distance when moving in where speed is super slow
#define DRAWER_MOVING_OUT_DECELERATION_DISTANCE 70   // distance to the target position to start deceleration (max 255)

// The drawer accelerates in dependency of the time
#define DEFAULT_DRAWER_ACCELERATION 10

namespace drawer_controller
{

  class ElectricalDrawer : public IDrawer
  {
   public:
    ElectricalDrawer(uint32_t module_id,
                     uint8_t id,
                     std::shared_ptr<robast_can_msgs::CanDb> can_db,
                     std::shared_ptr<IGpioWrapper> gpio_wrapper,
                     const stepper_motor::StepperPinIdConfig& stepper_pin_id_config,
                     bool use_encoder,
                     uint8_t encoder_pin_a,
                     uint8_t encoder_pin_b,
                     uint8_t motor_driver_address);

    void init_electrical_lock(uint8_t pwr_open_lock_pin_id,
                              uint8_t pwr_close_lock_pin_id,
                              uint8_t sensor_lock_pin_id,
                              uint8_t sensor_drawer_closed_pin_id) override;

    void handle_electrical_lock_control() override;

    void can_in(robast_can_msgs::CanMessage msg) override;

    std::optional<robast_can_msgs::CanMessage> can_out() override;

    void update_state() override;

    void stop_motor();

    void start_motor();

    void init_motor();

   private:
    uint32_t _module_id;
    uint8_t _id;

    std::shared_ptr<IGpioWrapper> _gpio_wrapper;

    stepper_motor::StepperPinIdConfig _stepper_pin_id_config;

    std::unique_ptr<Encoder> _encoder;

    std::unique_ptr<ElectricalLock> _electrical_lock;

    std::unique_ptr<CanUtils> _can_utils;

    bool _is_drawer_moving_out;
    bool _triggered_deceleration_for_drawer_moving_out = false;
    bool _is_idling = true;
    bool _triggered_deceleration_for_drawer_moving_in = false;

    uint8_t _target_position_uint8 = 0;

    bool _stall_guard_enabled = false;

    std::unique_ptr<stepper_motor::Motor> _motor;

    bool _triggered_closing_lock_after_opening = false;

    /* FUNCTIONS */

    void handle_drawer_idle_state();

    void handle_drawer_active_state();

    void handle_drawer_just_opened() override;

    void handle_drawer_just_closed() override;

    void handle_drawer_moving_in();

    void handle_drawer_moving_out();

    void handle_decelerating_for_moving_in_drawer();

    void handle_decelerating_for_moving_out_drawer();

    void handle_finished_moving_in_drawer();

    void handle_finished_moving_out_drawer();

    void set_target_speed_and_direction(uint8_t target_speed);

    void unlock();

    void check_if_drawer_is_homed();

    void handle_electrical_drawer_task_msg(robast_can_msgs::CanMessage can_message);

    void debug_prints_moving_electrical_drawer();

    void debug_prints_electric_drawer_task(robast_can_msgs::CanMessage can_message);

    void debug_prints_drawer_lock(robast_can_msgs::CanMessage& can_message);
  };
}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_ELECTRICAL_DRAWER_HPP
