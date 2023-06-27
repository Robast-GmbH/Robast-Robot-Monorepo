#ifndef ELECTRICAL_DRAWER_HPP
#define ELECTRICAL_DRAWER_HPP
#include <ESP32Encoder.h>

#include "can/can_db.hpp"
#include "i_drawer.hpp"
#include "i_gpio_wrapper.hpp"
#include "lock.hpp"
#include "motor.hpp"

#define DRAWER_MAX_EXTENT                       50000
#define DRAWER_MAX_SPEED                        35000
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

    void init_lock(uint8_t pwr_open_lock_pin_id,
                   uint8_t pwr_close_lock_pin_id,
                   uint8_t sensor_lock_pin_id,
                   uint8_t sensor_drawer_closed_pin_id);

    void stop_motor();

    void start_motor();

    void init_motor();

    void can_in(robast_can_msgs::CanMessage msg) override;

    std::optional<robast_can_msgs::CanMessage> can_out() override;

    void update_state() override;

   private:
    uint32_t _module_id;
    uint8_t _id;
    std::shared_ptr<robast_can_msgs::CanDb> _can_db;

    std::shared_ptr<IGpioWrapper> _gpio_wrapper;

    stepper_motor::StepperPinIdConfig _stepper_pin_id_config;

    bool _use_encoder;
    std::unique_ptr<Lock> _lock;

    uint32_t _last_timestemp;
    uint32_t _start_ramp_up_timestamp;
    uint32_t _starting_speed_before_ramp;
    bool _speed_ramp_in_progress = false;

    int _pos = 0;
    uint8_t _target_position = 0;

    bool _stall_guard_enabled = false;

    // Please mind: Normaly you would not built a queue yourself and use xQueue from FreeRTOS or std::queue
    // But: std:queue did not work and just permanently threw expections that made the ESP32 reboot
    // To use xQueue you need to use xQueueCreate to create a queue and you need to specify the size, in bytes, required
    // to hold each item in the queue, which is not possible for the CanMessage class because it contains a vector of
    // CanSignals which has a different length depending on the CanMessage.
    // Therefor we built a queue with a vector, which should be fine in this case as the queue usually only contains one
    // or two feedback messages and is rarely used. Furthermore we try to keep it as efficient as possible and try to
    // to follow what is explained here: https://youtu.be/fHNmRkzxHWs?t=2541
    std::vector<robast_can_msgs::CanMessage> _feedback_msg_queue;
    uint8_t _head_of_feedback_msg_queue;

    std::unique_ptr<stepper_motor::Motor> _motor;

    std::unique_ptr<ESP32Encoder> _encoder;

    bool _triggered_closing_lock_after_opening = false;

    /* FUNCTIONS */

    int get_integrated_drawer_position(stepper_motor::Direction direction);

    void update_motor_speed();

    void unlock();

    void drawer_homing();

    void handle_electrical_drawer_task_msg(robast_can_msgs::CanMessage can_message);

    void create_electrical_drawer_feedback_msg();

    void update_position(stepper_motor::Direction direction);

    void check_if_motion_is_finished(stepper_motor::Direction direction);

    void handle_drawer_just_opened() override;

    void handle_drawer_just_closed() override;

    void create_drawer_feedback_can_msg();

    void add_element_to_feedback_msg_queue(robast_can_msgs::CanMessage feedback_msg);

    std::optional<robast_can_msgs::CanMessage> get_element_from_feedback_msg_queue();

    void debug_prints_moving_electrical_drawer();

    void debug_prints_electric_drawer_task(robast_can_msgs::CanMessage can_message);

    void debug_prints_drawer_lock(robast_can_msgs::CanMessage& can_message);
  };
}   // namespace drawer_controller

#endif
