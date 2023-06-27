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
                     uint8_t motor_driver_address)
        : _module_id{module_id},
          _id{id},
          _can_db{can_db},
          _gpio_wrapper{gpio_wrapper},
          _stepper_pin_id_config{stepper_pin_id_config},
          _use_encoder{use_encoder},
          _lock{std::make_unique<Lock>(gpio_wrapper)}
    {
      if (use_encoder)
      {
        _encoder = std::make_unique<ESP32Encoder>(true);
        ESP32Encoder::useInternalWeakPullResistors = UP;
        _encoder->attachFullQuad(encoder_pin_a, encoder_pin_b);
        _encoder->setCount(0);
      }

      _motor = std::make_unique<stepper_motor::Motor>(motor_driver_address, _gpio_wrapper, _stepper_pin_id_config);
      _feedback_msg_queue.clear();
      _head_of_feedback_msg_queue = 0;
    }

    void init_lock(uint8_t pwr_open_lock_pin_id,
                   uint8_t pwr_close_lock_pin_id,
                   uint8_t sensor_lock_pin_id,
                   uint8_t sensor_drawer_closed_pin_id)
    {
      _lock->initialize_lock(
          pwr_open_lock_pin_id, pwr_close_lock_pin_id, sensor_lock_pin_id, sensor_drawer_closed_pin_id);
    }

    void stop_motor()
    {
      _motor->set_active_speed(0);
    }

    void start_motor()
    {
      _motor->set_active_speed(1000);
    }

    void init_motor()
    {
      _motor->init();
    }

    void can_in(robast_can_msgs::CanMessage msg) override
    {
      if (msg.get_id() == CAN_ID_ELECTRICAL_DRAWER_TASK)
      {
        handle_electrical_drawer_task_msg(msg);
        debug_prints_electric_drawer_task(msg);
      }
      if (msg.get_id() == CAN_ID_DRAWER_UNLOCK)
      {
        Serial.print("Received request to unlock the lock!");
        _lock->unlock(_id);
        debug_prints_drawer_lock(msg);
      }
    }

    std::optional<robast_can_msgs::CanMessage> can_out() override
    {
      return get_element_from_feedback_msg_queue();
    }

    void update_state() override
    {
      _lock->handle_lock_control();
      _lock->handle_reading_sensors();

      update_motor_speed();

      handle_drawer_just_closed();

      if (_motor->get_active_speed() == 0)
      {
        return;
      }

      stepper_motor::Direction direction = _motor->get_direction();

      update_position(direction);

      debug_prints_moving_electrical_drawer();

      handle_drawer_just_opened();

      check_if_motion_is_finished(direction);
    }

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

    int get_integrated_drawer_position(stepper_motor::Direction direction)
    {
      int integrated_position = 0;
      uint32_t current_timestemp = millis();

      if (direction == stepper_motor::counter_clockwise)
      {
        integrated_position = (current_timestemp - _last_timestemp) * _motor->get_active_speed() /
                              DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN;
      }
      else
      {
        integrated_position = (current_timestemp - _last_timestemp) * _motor->get_active_speed() /
                              DRAWER_POSITION_OPEN_LOOP_INTEGRAL_GAIN;
        integrated_position *= -1;
      }
      _last_timestemp = current_timestemp;

      return integrated_position;
    }

    void update_motor_speed()
    {
      uint32_t active_speed = _motor->get_active_speed();
      uint32_t target_speed = _motor->get_target_speed();

      if (active_speed == target_speed)
      {
        return;
      }
      else if (!_speed_ramp_in_progress)
      {
        _start_ramp_up_timestamp = millis();
        _starting_speed_before_ramp = active_speed;
        _speed_ramp_in_progress = true;
      }

      uint32_t current_timestemp = millis();
      uint32_t dt_since_start = current_timestemp - _start_ramp_up_timestamp;

      if (dt_since_start < DRAWER_ACCELERATION_TIME_IN_US)
      {
        uint32_t test1 = (dt_since_start / DRAWER_ACCELERATION_TIME_IN_US);

        uint32_t new_active_speed =
            _starting_speed_before_ramp +
            ((dt_since_start * (target_speed - _starting_speed_before_ramp)) / DRAWER_ACCELERATION_TIME_IN_US);
        _motor->set_active_speed(new_active_speed);
      }
      else
      {
        // The ramp-up time has elapsed, set the motor speed to the target speed directly
        _motor->set_active_speed(target_speed);
        _speed_ramp_in_progress = false;
        Serial.printf(" The ramp-up time has elapsed, set the motor speed to the target speed directly!\n");
      }
    }

    void unlock()
    {
      if (_lock->is_drawer_opening_in_progress())
      {
        Serial.printf("Drawer%d opening is already in progress, so lock won't be opened again!\n", _id);
      }
      else
      {
        _lock->set_open_lock_current_step(true);
        _lock->set_timestamp_last_lock_change();
        _lock->set_drawer_opening_is_in_progress(true);
      }
    }

    void drawer_homing()
    {
      if (_lock->is_endstop_switch_pushed())
      {
        _pos = 0;
      }
    }

    void handle_electrical_drawer_task_msg(robast_can_msgs::CanMessage can_message)
    {
      _target_position = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_TARGET_POSITION).get_data();
      uint8_t speed = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_SPEED).get_data();
      _stall_guard_enabled = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data() ==
                                     CAN_DATA_ELECTRICAL_DRAWER_STALL_GUARD_ENABLED
                                 ? true
                                 : false;

      uint32_t normed_target_speed = (speed * DRAWER_MAX_SPEED) / UINT8_MAX;

      // motor_->setStallGuard(stall_guard_enabled_);

      drawer_homing();

      if (_target_position == _pos)
      {
        create_electrical_drawer_feedback_msg();
        return;
      }

      bool is_drawer_retracted = _lock->is_endstop_switch_pushed();
      bool is_lock_open = _lock->is_lock_switch_pushed();

      if ((is_drawer_retracted && is_lock_open) || (!is_drawer_retracted))
      {
        _motor->set_target_speed(normed_target_speed);
        if (_target_position < _pos)
        {
          _motor->set_direction(stepper_motor::clockwise);
        }
        else
        {
          _motor->set_direction(stepper_motor::counter_clockwise);
        }
        if (!_use_encoder)
        {
          _last_timestemp = millis();
        }
      }
      else
      {
        Serial.println(
            "The electrical drawer can't be moved because the lock is not opened or the drawer is already retracted!");
      }
    }

    void create_electrical_drawer_feedback_msg()
    {
      robast_can_msgs::CanMessage can_msg_electrical_drawer_feedback =
          _can_db->can_messages.at(CAN_MSG_ELECTRICAL_DRAWER_FEEDBACK);
      std::vector can_signals_electrical_drawer_feedback = can_msg_electrical_drawer_feedback.get_can_signals();

      can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(_module_id);
      can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(_id);

      const bool is_endstop_switch_pushed = _lock->is_endstop_switch_pushed();
      can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);

      const bool is_lock_switch_pushed = _lock->is_lock_switch_pushed();
      can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

      const bool is_drawer_stall_guard_triggered = _motor->get_is_stalled();
      can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED)
          .set_data(is_lock_switch_pushed);

      uint8_t normed_current_position = (_pos * 255) / DRAWER_MAX_EXTENT;
      can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_POSITION).set_data(normed_current_position);

      can_msg_electrical_drawer_feedback.set_can_signals(can_signals_electrical_drawer_feedback);

      Serial.printf("Creating feedback_msg for module_id %d, drawer_id %d and position: %d\n",
                    _module_id,
                    _id,
                    normed_current_position);

      add_element_to_feedback_msg_queue(can_msg_electrical_drawer_feedback);
    }

    void update_position(stepper_motor::Direction direction)
    {
      if (_use_encoder)
      {
        _pos = _encoder->getCount();
      }
      else
      {
        _pos += get_integrated_drawer_position(direction);
      }
    }

    void check_if_motion_is_finished(stepper_motor::Direction direction)
    {
      int normed_target_position = (_target_position / 255.0) * DRAWER_MAX_EXTENT;
      if ((direction == stepper_motor::counter_clockwise) && (_pos >= normed_target_position))
      {
        Serial.printf("Current position: %d, Target position: %d\n", _pos, normed_target_position);
        _motor->set_target_speed(0);
        _motor->set_active_speed(0);
        create_electrical_drawer_feedback_msg();
      }
      else if ((direction == stepper_motor::clockwise) && (_pos <= normed_target_position))
      {
        Serial.printf("Current position: %d, Target position: %d\n", _pos, normed_target_position);

        _motor->set_target_speed(0);
        _motor->set_active_speed(0);
        if (_use_encoder)
        {
          _encoder->setCount(0);
        }
        _pos = 0;

        create_electrical_drawer_feedback_msg();
      }
    }

    void handle_drawer_just_opened() override
    {
      bool is_drawer_retracted = _lock->is_endstop_switch_pushed();
      if (_lock->is_drawer_opening_in_progress() && !is_drawer_retracted && !_triggered_closing_lock_after_opening)
      {
        _lock->set_open_lock_current_step(
            false);   // this makes sure the lock automatically closes as soon as the drawer is opened
        _triggered_closing_lock_after_opening =
            true;     // this makes sure, closing the lock is only triggered once and not permanently
        Serial.println("Triggered closing the lock because drawer is not retracted anymore!");
      }
    }

    void handle_drawer_just_closed() override
    {
      bool is_drawer_closed = _lock->is_endstop_switch_pushed() && !_lock->is_lock_switch_pushed();

      if (_lock->is_drawer_opening_in_progress() && is_drawer_closed && _triggered_closing_lock_after_opening)
      {
        _lock->set_drawer_opening_is_in_progress(false);
        _triggered_closing_lock_after_opening = false;   // reset this flag for the next opening of the drawer
        create_drawer_feedback_can_msg();
        Serial.println("Created a drawer feedback can message because the drawer just closed!");
      }
    }

    void create_drawer_feedback_can_msg()
    {
      robast_can_msgs::CanMessage can_msg_drawer_feedback = this->_can_db->can_messages.at(CAN_MSG_DRAWER_FEEDBACK);
      std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

      can_signals_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(_module_id);
      can_signals_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(_id);

      const bool is_endstop_switch_pushed = _lock->is_endstop_switch_pushed();
      can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);

      const bool is_lock_switch_pushed = _lock->is_lock_switch_pushed();
      can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

      can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

      add_element_to_feedback_msg_queue(can_msg_drawer_feedback);
    }

    void add_element_to_feedback_msg_queue(robast_can_msgs::CanMessage feedback_msg)
    {
      _feedback_msg_queue.push_back(feedback_msg);
    }

    std::optional<robast_can_msgs::CanMessage> get_element_from_feedback_msg_queue()
    {
      uint8_t num_of_msgs_in_queue = _feedback_msg_queue.size();
      if (num_of_msgs_in_queue == 0)
      {
        return {};
      }

      if (_head_of_feedback_msg_queue == (num_of_msgs_in_queue - 1))
      {
        robast_can_msgs::CanMessage feedback_can_msg = _feedback_msg_queue[_head_of_feedback_msg_queue];
        _feedback_msg_queue.clear();
        _head_of_feedback_msg_queue = 0;
        return feedback_can_msg;
      }
      else
      {
        return _feedback_msg_queue[_head_of_feedback_msg_queue++];
      }
    }

    void debug_prints_moving_electrical_drawer()
    {
      int normed_target_position = (_target_position / 255.0) * DRAWER_MAX_EXTENT;
      Serial.printf(
          "Current position: % d, Target Position: %d, Current Speed: %d, Target Speed: %d, _last_timestemp: %d, "
          "millis(): %d\n",
          _pos,
          normed_target_position,
          _motor->get_active_speed(),
          _motor->get_target_speed(),
          _last_timestemp,
          millis());
    }

    void debug_prints_electric_drawer_task(robast_can_msgs::CanMessage can_message)
    {
      Serial.print("Standard ID: ");
      Serial.print(can_message.get_id(), HEX);
      Serial.print(" rx_dlc: ");
      Serial.print(can_message.get_dlc(), DEC);
      Serial.print(" MODULE ID: ");
      Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
      Serial.print(" DRAWER ID: ");
      Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
      Serial.print(" GOTO POSITION: ");
      Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_TARGET_POSITION).get_data(), DEC);
      Serial.print(" SPEED MODE: ");
      Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_SPEED).get_data(), DEC);
      Serial.print(" STALL GUARD ENABLE: ");
      Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data(), DEC);
    }

    void debug_prints_drawer_lock(robast_can_msgs::CanMessage& can_message)
    {
      Serial.print("Standard ID: ");
      Serial.print(can_message.get_id(), HEX);
      Serial.print(" rx_dlc: ");
      Serial.print(can_message.get_dlc(), DEC);
      Serial.print(" MODULE ID: ");
      Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
      Serial.print(" DRAWER ID: ");
      Serial.println(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
    }
  };
}   // namespace drawer_controller

#endif
