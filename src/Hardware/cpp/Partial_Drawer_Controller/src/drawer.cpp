#include "drawer/drawer.hpp"

namespace drawer_controller
{

  Drawer::Drawer(uint32_t module_id,
                 uint8_t id,
                 std::shared_ptr<robast_can_msgs::CanDb> can_db,
                 std::shared_ptr<IGpioWrapper> gpio_wrapper)
      : _module_id{module_id},
        _id{id},
        _gpio_wrapper{gpio_wrapper},
        _electrical_lock{std::make_unique<ElectricalLock>(gpio_wrapper)},
        _can_utils{std::make_unique<CanUtils>(can_db)} {};

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
    return _can_utils->get_element_from_feedback_msg_queue();
  };

  void Drawer::update_state()
  {
    handle_electrical_lock_control();
    handle_drawer_just_opened();
    handle_drawer_just_closed();
  }

  void Drawer::handle_electrical_lock_control()
  {
    _electrical_lock->handle_lock_control();

    if (_electrical_lock->is_drawer_auto_close_timeout_triggered())
    {
      _can_utils->handle_error_feedback_msg(_module_id, _id, CAN_DATA_ERROR_CODE_TIMEOUT_DRAWER_NOT_OPENED);
      _electrical_lock->set_drawer_auto_close_timeout_triggered(false);
    }

    _electrical_lock->update_sensor_values();
  }

  void Drawer::handle_drawer_just_opened()
  {
    bool is_drawer_open = !_electrical_lock->is_endstop_switch_pushed();
    if (_electrical_lock->is_drawer_opening_in_progress() && is_drawer_open && !_drawer_open_feedback_can_msg_sent)
    {
      _electrical_lock->set_open_lock_current_step(
        false);   // this makes sure the lock automatically closes as soon as the drawer is opened
      _drawer_open_feedback_can_msg_sent = true;   // makes sure the feedback msg is only sent once
      _can_utils->handle_drawer_feedback_msg(
        _module_id, _id, _electrical_lock->is_endstop_switch_pushed(), _electrical_lock->is_lock_switch_pushed());
    }
  }

  void Drawer::handle_drawer_just_closed()
  {
    bool is_drawer_closed = _electrical_lock->is_endstop_switch_pushed() && !_electrical_lock->is_lock_switch_pushed();
    if (_electrical_lock->is_drawer_opening_in_progress() && is_drawer_closed && _drawer_open_feedback_can_msg_sent)
    {
      _electrical_lock->set_drawer_opening_is_in_progress(false);
      _drawer_open_feedback_can_msg_sent = false;   // reset this flag for the next opening of the drawer
      _can_utils->handle_drawer_feedback_msg(
        _module_id, _id, _electrical_lock->is_endstop_switch_pushed(), _electrical_lock->is_lock_switch_pushed());
    }
  }

  void Drawer::debug_prints_drawer_lock(robast_can_msgs::CanMessage &can_message)
  {
    debug_print("Received open lock CAN message with standard ID: ");
    debug_print_with_base(can_message.get_id(), HEX);
    debug_print(" rx_dlc: ");
    debug_print_with_base(can_message.get_dlc(), DEC);
    debug_print(" MODULE ID: ");
    debug_print_with_base(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
    debug_print(" DRAWER ID: ");
    debug_println_with_base(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
  }

}   // namespace drawer_controller