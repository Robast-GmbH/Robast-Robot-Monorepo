#include "can_toolbox/can_utils.hpp"

using CanMessage = robast_can_msgs::CanMessage;
namespace error_feedback = robast_can_msgs::can_signal::id::error_feedback;
namespace drawer_feedback = robast_can_msgs::can_signal::id::drawer_feedback;
namespace e_drawer_feedback = robast_can_msgs::can_signal::id::e_drawer_feedback;

namespace drawer_controller
{
  CanUtils::CanUtils(const std::shared_ptr<robast_can_msgs::CanDb> can_db)
      : _can_db{can_db}, _feedback_can_msg_queue{std::make_unique<Queue<CanMessage>>()}
  {
  }

  std::optional<CanMessage> CanUtils::get_element_from_feedback_msg_queue() const
  {
    return _feedback_can_msg_queue->get_element_from_queue();
  }

  void CanUtils::handle_error_feedback_msg(const uint32_t module_id, const uint8_t id, const uint8_t error_code) const
  {
    CanMessage drawer_closed_feedback_msg = create_error_feedback_msg(module_id, id, error_code);
    _feedback_can_msg_queue->add_element_to_queue(drawer_closed_feedback_msg);
  }

  void CanUtils::handle_drawer_feedback_msg(const uint32_t module_id,
                                            const uint8_t id,
                                            const bool is_endstop_switch_pushed,
                                            const bool is_lock_switch_pushed) const
  {
    CanMessage drawer_closed_feedback_msg =
      create_drawer_feedback_msg(module_id, id, is_endstop_switch_pushed, is_lock_switch_pushed);
    _feedback_can_msg_queue->add_element_to_queue(drawer_closed_feedback_msg);
  }

  void CanUtils::handle_electrical_drawer_feedback_msg(const uint32_t module_id,
                                                       const uint8_t id,
                                                       const bool is_endstop_switch_pushed,
                                                       const bool is_lock_switch_pushed,
                                                       const bool is_drawer_stall_guard_triggered,
                                                       const uint8_t normed_current_position,
                                                       const bool is_push_to_close_triggered) const
  {
    CanMessage electrical_drawer_feedback_msg = create_electrical_drawer_feedback_msg(module_id,
                                                                                      id,
                                                                                      is_endstop_switch_pushed,
                                                                                      is_lock_switch_pushed,
                                                                                      is_drawer_stall_guard_triggered,
                                                                                      normed_current_position,
                                                                                      is_push_to_close_triggered);
    _feedback_can_msg_queue->add_element_to_queue(electrical_drawer_feedback_msg);
  }

  CanMessage CanUtils::create_error_feedback_msg(const uint32_t module_id,
                                                 const uint8_t id,
                                                 const uint8_t error_code) const
  {
    CanMessage can_msg_error_feedback = _can_db->can_messages.at(robast_can_msgs::can_msg::ERROR_FEEDBACK);
    std::vector can_signals_error_feedback = can_msg_error_feedback.get_can_signals();

    can_signals_error_feedback.at(error_feedback::MODULE_ID).set_data(module_id);
    can_signals_error_feedback.at(error_feedback::DRAWER_ID).set_data(id);
    can_signals_error_feedback.at(error_feedback::ERROR_CODE).set_data(error_code);

    can_msg_error_feedback.set_can_signals(can_signals_error_feedback);

    return can_msg_error_feedback;
  }

  CanMessage CanUtils::create_drawer_feedback_msg(const uint32_t module_id,
                                                  const uint8_t id,
                                                  const bool is_endstop_switch_pushed,
                                                  const bool is_lock_switch_pushed) const
  {
    CanMessage can_msg_drawer_feedback = _can_db->can_messages.at(robast_can_msgs::can_msg::DRAWER_FEEDBACK);
    std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

    can_signals_drawer_feedback.at(drawer_feedback::MODULE_ID).set_data(module_id);
    can_signals_drawer_feedback.at(drawer_feedback::DRAWER_ID).set_data(id);
    can_signals_drawer_feedback.at(drawer_feedback::IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);
    can_signals_drawer_feedback.at(drawer_feedback::IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

    can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

    return can_msg_drawer_feedback;
  }

  CanMessage CanUtils::create_electrical_drawer_feedback_msg(const uint32_t module_id,
                                                             const uint8_t id,
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
    can_signals_electrical_drawer_feedback.at(e_drawer_feedback::DRAWER_ID).set_data(id);
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
}   // namespace drawer_controller