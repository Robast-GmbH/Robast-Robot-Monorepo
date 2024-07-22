#ifndef DRAWER_CONTROLLER_ELECTRICAL_DRAWER_HPP
#define DRAWER_CONTROLLER_ELECTRICAL_DRAWER_HPP

#include <Arduino.h>

#include <memory>
#include <optional>

#include "can_toolbox/can_utils.hpp"
#include "interfaces/i_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "lock/electrical_drawer_lock.hpp"
#include "motor/encoder.hpp"
#include "motor/motor.hpp"
#include "switch/switch.hpp"
#include "utils/e_drawer_task.hpp"
#include "utils/queue.hpp"

#define DRAWER_MAX_SPEED              35000
#define DRAWER_HOMING_SPEED           300
#define DRAWER_TARGET_HOMING_POSITION 0
#define STALL_GUARD_DISABLED          0
#define IS_HOMING                     true

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
    ElectricalDrawer(const uint32_t module_id,
                     const uint8_t id,
                     const std::shared_ptr<robast_can_msgs::CanDb> can_db,
                     const std::shared_ptr<IGpioWrapper> gpio_wrapper,
                     const stepper_motor::StepperPinIdConfig &stepper_pin_id_config,
                     const bool use_encoder,
                     const uint8_t encoder_pin_a,
                     const uint8_t encoder_pin_b,
                     const uint8_t motor_driver_address,
                     const bool shaft_direction_is_inverted,
                     const std::shared_ptr<Switch> endstop_switch,
                     const std::optional<std::shared_ptr<ElectricalDrawerLock>> electrical_drawer_lock);

    void init() const;

    std::optional<robast_can_msgs::CanMessage> can_out() override;

    void update_state() override;

    void unlock();

    void add_e_drawer_task_to_queue(const EDrawerTask &e_drawer_task);

    void stop_motor() const;

    void start_motor() const;

   private:
    const uint32_t _module_id;
    const uint8_t _id;

    const std::shared_ptr<IGpioWrapper> _gpio_wrapper;

    const stepper_motor::StepperPinIdConfig _stepper_pin_id_config;

    const std::unique_ptr<Encoder> _encoder;

    const std::shared_ptr<Switch> _endstop_switch;

    // optional because the lock is not always installed (e.g. in the partial drawer)
    const std::optional<std::shared_ptr<ElectricalDrawerLock>> _electrical_drawer_lock;

    const std::unique_ptr<CanUtils> _can_utils;

    const std::unique_ptr<stepper_motor::Motor> _motor;

    std::unique_ptr<Queue<EDrawerTask>> _e_drawer_task_queue;

    bool _drawer_was_homed_once = false;

    bool _is_drawer_moving_out;
    bool _triggered_deceleration_for_drawer_moving_out = false;
    bool _is_idling = true;
    bool _triggered_deceleration_for_drawer_moving_in = false;

    uint8_t _target_position_uint8 = 0;

    bool _triggered_closing_lock_after_opening = false;

    /* FUNCTIONS */

    void init_motor() const;

    void handle_drawer_idle_state();

    void handle_drawer_active_state();

    void start_normal_drawer_movement(const uint8_t target_speed);

    void start_homing_movement(const uint8_t target_speed);

    bool check_and_handle_initial_drawer_homing();

    void handle_drawer_just_opened() override;

    void handle_drawer_just_closed() override;

    void handle_electrical_drawer_lock_control() override;

    void handle_drawer_moving_in();

    void handle_drawer_moving_out();

    void handle_decelerating_for_moving_in_drawer();

    void handle_decelerating_for_moving_out_drawer();

    void handle_finished_moving_in_drawer();

    void handle_finished_moving_out_drawer();

    bool handle_motor_stall_guard_and_return_status();

    void set_target_speed_and_direction(uint8_t target_speed);

    void check_if_drawer_is_homed();

    void debug_prints_moving_electrical_drawer();

    uint32_t get_normed_target_speed_uint32(const uint8_t target_speed) const;

    uint8_t get_normed_target_speed_uint8(const uint32_t target_speed) const;
  };
}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_ELECTRICAL_DRAWER_HPP
