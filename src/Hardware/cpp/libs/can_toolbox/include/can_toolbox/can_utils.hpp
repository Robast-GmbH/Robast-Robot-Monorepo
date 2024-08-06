#ifndef DRAWER_CONTROLLER_CAN_UTILS_HPP
#define DRAWER_CONTROLLER_CAN_UTILS_HPP

#include <Arduino.h>

#include <memory>
#include <optional>

#include "can/can_db.hpp"
#include "can/can_message.hpp"
#include "debug/debug.hpp"
#include "utils/queue.hpp"

namespace drawer_controller
{
  class CanUtils
  {
   public:
    CanUtils(const std::shared_ptr<robast_can_msgs::CanDb> can_db);

    std::optional<robast_can_msgs::CanMessage> get_element_from_feedback_msg_queue() const;

    void enqueue_error_feedback_msg(const uint32_t module_id, const uint8_t drawer_id, uint8_t error_code) const;

    void enqueue_drawer_feedback_msg(const uint32_t module_id,
                                     const uint8_t drawer_id,
                                     const bool is_endstop_switch_pushed,
                                     const bool is_lock_switch_pushed) const;

    void enqueue_e_drawer_feedback_msg(const uint32_t module_id,
                                       const uint8_t drawer_id,
                                       const bool is_endstop_switch_pushed,
                                       const bool is_lock_switch_pushed,
                                       const bool is_drawer_stall_guard_triggered,
                                       const uint8_t normed_current_position,
                                       const bool is_push_to_close_triggered) const;

   private:
    const std::unique_ptr<Queue<robast_can_msgs::CanMessage>> _feedback_can_msg_queue;

    const std::shared_ptr<robast_can_msgs::CanDb> _can_db;

    robast_can_msgs::CanMessage create_error_feedback_msg(const uint32_t module_id,
                                                          const uint8_t drawer_id,
                                                          const uint8_t error_code) const;

    robast_can_msgs::CanMessage create_drawer_feedback_msg(const uint32_t module_id,
                                                           const uint8_t drawer_id,
                                                           const bool is_endstop_switch_pushed,
                                                           const bool is_lock_switch_pushed) const;

    robast_can_msgs::CanMessage create_e_drawer_feedback_msg(const uint32_t module_id,
                                                             const uint8_t drawer_id,
                                                             const bool is_endstop_switch_pushed,
                                                             const bool is_lock_switch_pushed,
                                                             const bool is_drawer_stall_guard_triggered,
                                                             const uint8_t normed_current_position,
                                                             const bool is_push_to_close_triggered) const;
  };
}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_CAN_UTILS_HPP
