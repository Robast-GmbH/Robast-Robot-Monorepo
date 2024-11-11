#include "can_toolbox/can_utils.hpp"

using CanMessage = robast_can_msgs::CanMessage;
namespace error_feedback = robast_can_msgs::can_signal::id::error_feedback;
namespace drawer_feedback = robast_can_msgs::can_signal::id::drawer_feedback;
namespace e_drawer_feedback = robast_can_msgs::can_signal::id::e_drawer_feedback;
namespace electrical_drawer_motor_control = robast_can_msgs::can_signal::id::electrical_drawer_motor_control;

namespace can_toolbox
{
  CanUtils::CanUtils(const std::shared_ptr<robast_can_msgs::CanDb> can_db) : _can_db{can_db}
  {
  }

  std::optional<CanMessage> CanUtils::get_element_from_feedback_msg_queue() const
  {
    return _feedback_can_msg_queue->dequeue();
  }

  void CanUtils::enqueue_error_feedback_msg(const uint32_t module_id,
                                            const uint8_t drawer_id,
                                            const uint8_t error_code) const
  {
    const CanMessage error_feedback_msg = create_error_feedback_msg(module_id, drawer_id, error_code);
    _feedback_can_msg_queue->enqueue(error_feedback_msg);
  }

  void CanUtils::enqueue_drawer_feedback_msg(const uint32_t module_id,
                                             const uint8_t drawer_id,
                                             const bool is_endstop_switch_pushed,
                                             const bool is_lock_switch_pushed) const
  {
    const CanMessage drawer_closed_feedback_msg =
      create_drawer_feedback_msg(module_id, drawer_id, is_endstop_switch_pushed, is_lock_switch_pushed);
    _feedback_can_msg_queue->enqueue(drawer_closed_feedback_msg);
  }

  void CanUtils::enqueue_e_drawer_feedback_msg(const uint32_t module_id,
                                               const uint8_t drawer_id,
                                               const bool is_endstop_switch_pushed,
                                               const bool is_lock_switch_pushed,
                                               const bool is_drawer_stall_guard_triggered,
                                               const uint8_t normed_current_position,
                                               const bool is_push_to_close_triggered) const
  {
    const CanMessage electrical_drawer_feedback_msg = create_e_drawer_feedback_msg(module_id,
                                                                                   drawer_id,
                                                                                   is_endstop_switch_pushed,
                                                                                   is_lock_switch_pushed,
                                                                                   is_drawer_stall_guard_triggered,
                                                                                   normed_current_position,
                                                                                   is_push_to_close_triggered);
    _feedback_can_msg_queue->enqueue(electrical_drawer_feedback_msg);
  }

  void CanUtils::enqueue_e_drawer_motor_control_msg(const uint32_t module_id,
                                                    const uint8_t motor_id,
                                                    const bool enable_motor,
                                                    const bool confirm_control_change) const
  {
    const CanMessage electrical_drawer_motor_control_msg =
      create_e_drawer_motor_control_msg(module_id, motor_id, enable_motor, confirm_control_change);
    _feedback_can_msg_queue->enqueue(electrical_drawer_motor_control_msg);
  }

  void CanUtils::enqueue_heartbeat_msg(const uint32_t module_id) const
  {
    _feedback_can_msg_queue->enqueue(create_heartbeat_msg(module_id));
  }

  CanMessage CanUtils::create_error_feedback_msg(const uint32_t module_id,
                                                 const uint8_t drawer_id,
                                                 const uint8_t error_code) const
  {
    CanMessage can_msg_error_feedback = _can_db->can_messages.at(robast_can_msgs::can_msg::ERROR_FEEDBACK);
    std::vector can_signals_error_feedback = can_msg_error_feedback.get_can_signals();

    can_signals_error_feedback.at(error_feedback::MODULE_ID).set_data(module_id);
    can_signals_error_feedback.at(error_feedback::DRAWER_ID).set_data(drawer_id);
    can_signals_error_feedback.at(error_feedback::ERROR_CODE).set_data(error_code);

    can_msg_error_feedback.set_can_signals(can_signals_error_feedback);

    return can_msg_error_feedback;
  }

  CanMessage CanUtils::create_drawer_feedback_msg(const uint32_t module_id,
                                                  const uint8_t drawer_id,
                                                  const bool is_endstop_switch_pushed,
                                                  const bool is_lock_switch_pushed) const
  {
    CanMessage can_msg_drawer_feedback = _can_db->can_messages.at(robast_can_msgs::can_msg::DRAWER_FEEDBACK);
    std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

    can_signals_drawer_feedback.at(drawer_feedback::MODULE_ID).set_data(module_id);
    can_signals_drawer_feedback.at(drawer_feedback::DRAWER_ID).set_data(drawer_id);
    can_signals_drawer_feedback.at(drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);
    can_signals_drawer_feedback.at(drawer_feedback::IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

    can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

    return can_msg_drawer_feedback;
  }

  CanMessage CanUtils::create_e_drawer_feedback_msg(const uint32_t module_id,
                                                    const uint8_t drawer_id,
                                                    const bool is_endstop_switch_pushed,
                                                    const bool is_lock_switch_pushed,
                                                    const bool is_drawer_stall_guard_triggered,
                                                    const uint8_t normed_current_position,
                                                    const bool is_push_to_close_triggered) const
  {
    CanMessage can_msg_electrical_drawer_feedback =
      _can_db->can_messages.at(robast_can_msgs::can_msg::ELECTRICAL_DRAWER_FEEDBACK);
    std::vector can_signals_electrical_drawer_feedback = can_msg_electrical_drawer_feedback.get_can_signals();

    can_signals_electrical_drawer_feedback.at(e_drawer_feedback::MODULE_ID).set_data(module_id);
    can_signals_electrical_drawer_feedback.at(e_drawer_feedback::DRAWER_ID).set_data(drawer_id);
    can_signals_electrical_drawer_feedback.at(e_drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED)
      .set_data(is_endstop_switch_pushed);
    can_signals_electrical_drawer_feedback.at(e_drawer_feedback::IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);
    can_signals_electrical_drawer_feedback.at(e_drawer_feedback::DRAWER_IS_STALL_GUARD_TRIGGERED)
      .set_data(is_drawer_stall_guard_triggered);
    can_signals_electrical_drawer_feedback.at(e_drawer_feedback::DRAWER_POSITION).set_data(normed_current_position);
    can_signals_electrical_drawer_feedback.at(e_drawer_feedback::IS_PUSH_TO_CLOSE_TRIGGERED)
      .set_data(is_push_to_close_triggered);

    can_msg_electrical_drawer_feedback.set_can_signals(can_signals_electrical_drawer_feedback);

    return can_msg_electrical_drawer_feedback;
  }

  CanMessage CanUtils::create_e_drawer_motor_control_msg(const uint32_t module_id,
                                                         const uint8_t motor_id,
                                                         const bool enable_motor,
                                                         const bool confirm_control_change) const
  {
    CanMessage can_msg_electrical_drawer_motor_control =
      _can_db->can_messages.at(robast_can_msgs::can_msg::ELECTRICAL_DRAWER_MOTOR_CONTROL);
    std::vector can_signals_electrical_drawer_motor_control = can_msg_electrical_drawer_motor_control.get_can_signals();

    can_signals_electrical_drawer_motor_control.at(electrical_drawer_motor_control::MODULE_ID).set_data(module_id);
    can_signals_electrical_drawer_motor_control.at(electrical_drawer_motor_control::MOTOR_ID).set_data(motor_id);
    can_signals_electrical_drawer_motor_control.at(electrical_drawer_motor_control::ENABLE_MOTOR)
      .set_data(enable_motor);
    can_signals_electrical_drawer_motor_control.at(electrical_drawer_motor_control::CONFIRM_CONTROL_CHANGE)
      .set_data(confirm_control_change);

    can_msg_electrical_drawer_motor_control.set_can_signals(can_signals_electrical_drawer_motor_control);

    return can_msg_electrical_drawer_motor_control;
  }

  CanMessage CanUtils::create_heartbeat_msg(const uint32_t module_id) const
  {
    CanMessage can_msg_heartbeat = _can_db->can_messages.at(robast_can_msgs::can_msg::HEARTBEAT);
    std::vector can_signals_heartbeat = can_msg_heartbeat.get_can_signals();

    can_signals_heartbeat.at(robast_can_msgs::can_signal::id::heartbeat::MODULE_ID).set_data(module_id);

    can_msg_heartbeat.set_can_signals(can_signals_heartbeat);

    return can_msg_heartbeat;
  }
}   // namespace can_toolbox