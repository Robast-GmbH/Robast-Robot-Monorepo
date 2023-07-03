#ifndef ELECTRICAL_DRAWER_HPP
#define ELECTRICAL_DRAWER_HPP

#include <ESP32Encoder.h>

#include <memory>

#include "can/can_db.hpp"
#include "can_utils.hpp"
#include "electrical_lock.hpp"
#include "i_drawer.hpp"
#include "i_gpio_wrapper.hpp"
#include "motor.hpp"

#define DRAWER_MAX_EXTENT                       85000
#define DRAWER_MAX_SPEED                        35000
#define DRAWER_HOMING_SPEED                     150
#define DRAWER_HOMING_EXTENT                    40   // this value determines the extent of the homing area (max 255)
#define DRAWER_ACCELERATION_TIME_IN_US          1000
#define DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN 1000

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

    bool _use_encoder;
    std::unique_ptr<ElectricalLock> _electrical_lock;

    std::unique_ptr<CanUtils> _can_utils;

    uint32_t _last_timestemp;

    bool _is_drawer_moving_out;
    bool _electrical_drawer_opening_in_progress = false;
    bool _homing_initialized = false;

    int32_t _current_position_int32 = 0;
    uint8_t _target_position_uint8 = 0;

    bool _stall_guard_enabled = false;

    std::unique_ptr<stepper_motor::Motor> _motor;

    std::unique_ptr<ESP32Encoder> _encoder;

    bool _triggered_closing_lock_after_opening = false;

    /* FUNCTIONS */

    void handle_drawer_just_opened() override;

    void handle_drawer_just_closed() override;

    int32_t get_integrated_drawer_position();

    void handle_drawer_moving_in(uint8_t normed_current_position_uint8);

    void initialize_homing();

    void set_target_speed_and_direction(uint8_t target_speed);

    void unlock();

    void check_if_drawer_is_homed();

    void handle_electrical_drawer_task_msg(robast_can_msgs::CanMessage can_message);

    void update_position();

    void check_if_motion_is_finished();

    uint8_t get_normed_current_position();

    void handle_electrical_drawer_feeback_msg();

    void handle_drawer_closed_feedback_msg();

    void debug_prints_moving_electrical_drawer();

    void debug_prints_electric_drawer_task(robast_can_msgs::CanMessage can_message);

    void debug_prints_drawer_lock(robast_can_msgs::CanMessage& can_message);
  };
}   // namespace drawer_controller

#endif
