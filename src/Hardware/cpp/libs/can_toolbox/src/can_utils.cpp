#include "can_toolbox/can_utils.hpp"

namespace drawer_controller
{
  CanUtils::CanUtils(const std::shared_ptr<robast_can_msgs::CanDb> can_db)
      : _can_db{can_db}, _feedback_can_msg_queue{std::make_unique<Queue<robast_can_msgs::CanMessage>>()}
  {
  }

  std::optional<robast_can_msgs::CanMessage> CanUtils::get_element_from_feedback_msg_queue() const
  {
    return _feedback_can_msg_queue->get_element_from_queue();
  }

  void CanUtils::handle_error_feedback_msg(const uint32_t module_id, const uint8_t id, const uint8_t error_code) const
  {
    robast_can_msgs::CanMessage drawer_closed_feedback_msg = create_error_feedback_msg(module_id, id, error_code);
    _feedback_can_msg_queue->add_element_to_queue(drawer_closed_feedback_msg);
  }

  void CanUtils::handle_drawer_feedback_msg(const uint32_t module_id,
                                            const uint8_t id,
                                            const bool is_endstop_switch_pushed,
                                            const bool is_lock_switch_pushed) const
  {
    robast_can_msgs::CanMessage drawer_closed_feedback_msg =
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
    robast_can_msgs::CanMessage electrical_drawer_feedback_msg =
      create_electrical_drawer_feedback_msg(module_id,
                                            id,
                                            is_endstop_switch_pushed,
                                            is_lock_switch_pushed,
                                            is_drawer_stall_guard_triggered,
                                            normed_current_position,
                                            is_push_to_close_triggered);
    _feedback_can_msg_queue->add_element_to_queue(electrical_drawer_feedback_msg);
  }

  robast_can_msgs::CanMessage CanUtils::create_error_feedback_msg(const uint32_t module_id,
                                                                  const uint8_t id,
                                                                  const uint8_t error_code) const
  {
    robast_can_msgs::CanMessage can_msg_error_feedback =
      _can_db->can_messages.at(robast_can_msgs::can_msg::ERROR_FEEDBACK);
    std::vector can_signals_error_feedback = can_msg_error_feedback.get_can_signals();

    can_signals_error_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(module_id);
    can_signals_error_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(id);
    can_signals_error_feedback.at(CAN_SIGNAL_ERROR_CODE).set_data(error_code);

    can_msg_error_feedback.set_can_signals(can_signals_error_feedback);

    return can_msg_error_feedback;
  }

  robast_can_msgs::CanMessage CanUtils::create_drawer_feedback_msg(const uint32_t module_id,
                                                                   const uint8_t id,
                                                                   const bool is_endstop_switch_pushed,
                                                                   const bool is_lock_switch_pushed) const
  {
    robast_can_msgs::CanMessage can_msg_drawer_feedback =
      _can_db->can_messages.at(robast_can_msgs::can_msg::DRAWER_FEEDBACK);
    std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

    can_signals_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(module_id);
    can_signals_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(id);
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);
    can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

    can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

    return can_msg_drawer_feedback;
  }

  robast_can_msgs::CanMessage CanUtils::create_electrical_drawer_feedback_msg(
    const uint32_t module_id,
    const uint8_t id,
    const bool is_endstop_switch_pushed,
    const bool is_lock_switch_pushed,
    const bool is_drawer_stall_guard_triggered,
    const uint8_t normed_current_position,
    const bool is_push_to_close_triggered) const
  {
    robast_can_msgs::CanMessage can_msg_electrical_drawer_feedback =
      _can_db->can_messages.at(robast_can_msgs::can_msg::ELECTRICAL_DRAWER_FEEDBACK);
    std::vector can_signals_electrical_drawer_feedback = can_msg_electrical_drawer_feedback.get_can_signals();

    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(module_id);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(id);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED)
      .set_data(is_lock_switch_pushed);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_POSITION).set_data(normed_current_position);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_PUSH_TO_CLOSE_TRIGGERED)
      .set_data(is_push_to_close_triggered);

    can_msg_electrical_drawer_feedback.set_can_signals(can_signals_electrical_drawer_feedback);

    return can_msg_electrical_drawer_feedback;
  }
}   // namespace drawer_controller