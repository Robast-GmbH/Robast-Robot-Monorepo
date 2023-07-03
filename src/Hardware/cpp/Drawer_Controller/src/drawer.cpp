#include "drawer.hpp"

namespace drawer_controller
{

  Drawer::Drawer(uint32_t module_id,
                 uint8_t id,
                 std::shared_ptr<robast_can_msgs::CanDb> can_db,
                 std::shared_ptr<IGpioWrapper> gpio_wrapper)
      : _module_id{module_id},
        _id{id},
        _can_db{can_db},
        _gpio_wrapper{gpio_wrapper},
        _electrical_lock{std::make_unique<ElectricalLock>(gpio_wrapper)} {};

  void Drawer::init_electrical_lock(uint8_t pwr_open_lock_pin_id,
                                    uint8_t pwr_close_lock_pin_id,
                                    uint8_t sensor_lock_pin_id,
                                    uint8_t sensor_drawer_closed_pin_id)
  {
    _electrical_lock->initialize_lock(
      pwr_open_lock_pin_id, pwr_close_lock_pin_id, sensor_lock_pin_id, sensor_drawer_closed_pin_id);
  }

  void Drawer::can_in(robast_can_msgs::CanMessage msg)
  {
    if (msg.get_id() == CAN_ID_DRAWER_UNLOCK)
    {
      _electrical_lock->unlock(_id);

      debug_prints_drawer_lock(msg);
    }
  }

  std::optional<robast_can_msgs::CanMessage> Drawer::can_out()
  {
    return _feedback_msg;
  };

  void Drawer::update_state()
  {
    _feedback_msg.reset();   // TODO@Jacob: Remove?
    handle_electrical_lock_control();
    handle_drawer_just_opened();
    handle_drawer_just_closed();
  }

  void Drawer::handle_electrical_lock_control()
  {
    _electrical_lock->handle_lock_control();

    if (_electrical_lock->is_drawer_auto_close_timeout_triggered())
    {
      // TODO@Jacob: Create error message
    }

    _electrical_lock->handle_reading_sensors();
  }

  void Drawer::create_drawer_feedback_can_msg()
  {
    robast_can_msgs::CanMessage can_msg_drawer_feedback = this->_can_db->can_messages.at(CAN_MSG_DRAWER_FEEDBACK);
    std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

    can_signals_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(_module_id);
    can_signals_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(_id);

    const bool is_endstop_switch_pushed = _electrical_lock->is_endstop_switch_pushed();
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);

    const bool is_lock_switch_pushed = _electrical_lock->is_lock_switch_pushed();
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

    can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

    _feedback_msg = can_msg_drawer_feedback;
  }

  void Drawer::handle_drawer_just_opened()
  {
    bool is_drawer_open = !_electrical_lock->is_endstop_switch_pushed();
    if (_electrical_lock->is_drawer_opening_in_progress() && is_drawer_open && !_drawer_open_feedback_can_msg_sent)
    {
      _electrical_lock->set_open_lock_current_step(
        false);   // this makes sure the lock automatically closes as soon as the drawer is opened
      _drawer_open_feedback_can_msg_sent = true;   // makes sure the feedback msg is only sent once
      create_drawer_feedback_can_msg();
    }
  }

  void Drawer::handle_drawer_just_closed()
  {
    bool is_drawer_closed = _electrical_lock->is_endstop_switch_pushed() && !_electrical_lock->is_lock_switch_pushed();
    if (_electrical_lock->is_drawer_opening_in_progress() && is_drawer_closed && _drawer_open_feedback_can_msg_sent)
    {
      _electrical_lock->set_drawer_opening_is_in_progress(false);
      _drawer_open_feedback_can_msg_sent = false;   // reset this flag for the next opening of the drawer
      create_drawer_feedback_can_msg();
    }
  }

  void Drawer::debug_prints_drawer_lock(robast_can_msgs::CanMessage &can_message)
  {
    Serial.print("Received open lock CAN message with standard ID: ");
    Serial.print(can_message.get_id(), HEX);
    Serial.print(" rx_dlc: ");
    Serial.print(can_message.get_dlc(), DEC);
    Serial.print(" MODULE ID: ");
    Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
    Serial.print(" DRAWER ID: ");
    Serial.println(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
  }

}   // namespace drawer_controller