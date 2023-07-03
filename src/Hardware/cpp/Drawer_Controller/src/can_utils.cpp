#include "can_utils.hpp"

namespace drawer_controller
{
  CanUtils::CanUtils(std::shared_ptr<robast_can_msgs::CanDb> can_db) : _can_db{can_db}
  {
    _feedback_msg_queue.clear();
    _head_of_feedback_msg_queue = 0;
  }

  void CanUtils::add_element_to_feedback_msg_queue(robast_can_msgs::CanMessage feedback_msg)
  {
    _feedback_msg_queue.push_back(feedback_msg);
  }

  std::optional<robast_can_msgs::CanMessage> CanUtils::get_element_from_feedback_msg_queue()
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

  robast_can_msgs::CanMessage CanUtils::create_drawer_feedback_msg(const uint32_t module_id,
                                                                   const uint8_t id,
                                                                   const bool is_endstop_switch_pushed,
                                                                   const bool is_lock_switch_pushed)
  {
    robast_can_msgs::CanMessage can_msg_drawer_feedback = _can_db->can_messages.at(CAN_MSG_DRAWER_FEEDBACK);
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
    uint8_t normed_current_position)
  {
    robast_can_msgs::CanMessage can_msg_electrical_drawer_feedback =
      _can_db->can_messages.at(CAN_MSG_ELECTRICAL_DRAWER_FEEDBACK);
    std::vector can_signals_electrical_drawer_feedback = can_msg_electrical_drawer_feedback.get_can_signals();

    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(module_id);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(id);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);
    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED)
      .set_data(is_lock_switch_pushed);

    can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_POSITION).set_data(normed_current_position);

    can_msg_electrical_drawer_feedback.set_can_signals(can_signals_electrical_drawer_feedback);

    return can_msg_electrical_drawer_feedback;
  }
}   // namespace drawer_controller