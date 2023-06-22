#ifndef DRAWER_CONTROLLER_DRAWER_HPP
#define DRAWER_CONTROLLER_DRAWER_HPP
#include "can/can_db.hpp"
#include "i_drawer.hpp"
#include "i_gpio_wrapper.hpp"
#include "lock.hpp"

namespace drawer_controller
{
  class Drawer : public IDrawer
  {
   public:
    Drawer(uint32_t module_id, uint8_t id, std::shared_ptr<IGpioWrapper> gpio_wrapper)
        : _module_id{module_id}, _id{id}, _gpio_wrapper{gpio_wrapper} {};

    void init_lock(uint8_t pwr_open_lock_pin_id,
                   uint8_t pwr_close_lock_pin_id,
                   uint8_t sensor_lock_pin_id,
                   uint8_t sensor_drawer_closed_pin_id)
    {
      _lock.initialize_lock(
          pwr_open_lock_pin_id, pwr_close_lock_pin_id, sensor_lock_pin_id, sensor_drawer_closed_pin_id);
    }

    void can_in(robast_can_msgs::CanMessage msg) override
    {
      debug_prints_drawer_lock(msg);
      if (msg.get_id() == CAN_ID_DRAWER_UNLOCK)
      {
        _lock.unlock(_id);

        debug_prints_drawer_lock(msg);
      }
    }

    std::optional<robast_can_msgs::CanMessage> can_out() override
    {
      return _feedback_msg;
    };

    void update_state() override
    {
      _lock.handle_lock_control();
      _lock.handle_reading_sensors();
      _feedback_msg.reset();
      handle_drawer_is_open_feedback();
      handle_drawer_is_closed_feedback();
    }

   private:
    uint32_t _module_id;
    uint8_t _id;
    std::shared_ptr<IGpioWrapper> _gpio_wrapper;

    robast_can_msgs::CanDb _can_db = robast_can_msgs::CanDb();

    std::optional<robast_can_msgs::CanMessage> _feedback_msg;

    Lock _lock = Lock(_gpio_wrapper);

    bool _drawer_open_feedback_can_msg_sent = false;

    void create_drawer_feedback_can_msg()
    {
      robast_can_msgs::CanMessage can_msg_drawer_feedback = this->_can_db.can_messages.at(CAN_MSG_DRAWER_FEEDBACK);
      std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

      can_signals_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(_module_id);
      can_signals_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(_id);

      const bool is_endstop_switch_pushed = _lock.is_endstop_switch_pushed();
      can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);

      const bool is_lock_switch_pushed = _lock.is_lock_switch_pushed();
      can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

      can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

      _feedback_msg = can_msg_drawer_feedback;
    }

    void handle_drawer_is_open_feedback()
    {
      bool is_drawer_open = !_lock.is_endstop_switch_pushed();
      if (_lock.is_drawer_opening_in_progress() && is_drawer_open && !_drawer_open_feedback_can_msg_sent)
      {
        _lock.set_open_lock_current_step(
            false);   // this makes sure the lock automatically closes as soon as the drawer is opened
        _drawer_open_feedback_can_msg_sent = true;   // makes sure the feedback msg is only sent once
        create_drawer_feedback_can_msg();
      }
    }

    void handle_drawer_is_closed_feedback()
    {
      bool is_drawer_closed = _lock.is_endstop_switch_pushed() && !_lock.is_lock_switch_pushed();
      if (_lock.is_drawer_opening_in_progress() && is_drawer_closed && _drawer_open_feedback_can_msg_sent)
      {
        _lock.set_drawer_opening_is_in_progress(false);
        _drawer_open_feedback_can_msg_sent = false;   // reset this flag for the next opening of the drawer
        create_drawer_feedback_can_msg();
      }
    }

    void debug_prints_drawer_lock(robast_can_msgs::CanMessage &can_message)
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
