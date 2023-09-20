#ifndef DRAWER_CONTROLLER_CAN_UTILS_HPP
#define DRAWER_CONTROLLER_CAN_UTILS_HPP

#include <Arduino.h>

#include <memory>
#include <optional>

#include "can/can_db.hpp"
#include "can/can_message.h"
#include "debug/debug.hpp"

namespace drawer_controller
{
  class CanUtils
  {
   public:
    CanUtils(std::shared_ptr<robast_can_msgs::CanDb> can_db);

    void add_element_to_feedback_msg_queue(robast_can_msgs::CanMessage feedback_msg);

    std::optional<robast_can_msgs::CanMessage> get_element_from_feedback_msg_queue();

    void handle_error_feedback_msg(const uint32_t module_id, const uint8_t id, uint8_t error_code);

    void handle_drawer_feedback_msg(const uint32_t module_id,
                                    const uint8_t id,
                                    const bool is_endstop_switch_pushed,
                                    const bool is_lock_switch_pushed);

    void handle_electrical_drawer_feedback_msg(const uint32_t module_id,
                                               const uint8_t id,
                                               const bool is_endstop_switch_pushed,
                                               const bool is_lock_switch_pushed,
                                               const bool is_drawer_stall_guard_triggered,
                                               uint8_t normed_current_position);

   private:
    // Please mind: Normaly you would not built a queue yourself and use xQueue from FreeRTOS or std::queue
    // But: std:queue did not work and just permanently threw expections that made the ESP32 reboot
    // To use xQueue you need to use xQueueCreate to create a queue and you need to specify the size, in bytes, required
    // to hold each item in the queue, which is not possible for the CanMessage class because it contains a vector of
    // CanSignals which has a different length depending on the CanMessage.
    // Therefore we built a queue with a vector, which should be fine in this case as the queue usually only contains
    // one or two feedback messages and is rarely used. Furthermore we try to keep it as efficient as possible and try
    // to follow what is explained here: https://youtu.be/fHNmRkzxHWs?t=2541
    std::vector<robast_can_msgs::CanMessage> _feedback_msg_queue;
    uint8_t _head_of_feedback_msg_queue;

    std::shared_ptr<robast_can_msgs::CanDb> _can_db;

    robast_can_msgs::CanMessage create_error_feedback_msg(const uint32_t module_id,
                                                          const uint8_t id,
                                                          uint8_t error_code);

    robast_can_msgs::CanMessage create_drawer_feedback_msg(const uint32_t module_id,
                                                           const uint8_t id,
                                                           const bool is_endstop_switch_pushed,
                                                           const bool is_lock_switch_pushed);

    robast_can_msgs::CanMessage create_electrical_drawer_feedback_msg(const uint32_t module_id,
                                                                      const uint8_t id,
                                                                      const bool is_endstop_switch_pushed,
                                                                      const bool is_lock_switch_pushed,
                                                                      const bool is_drawer_stall_guard_triggered,
                                                                      uint8_t normed_current_position);
  };
}   // namespace drawer_controller
#endif   // DRAWER_CONTROLLER_CAN_UTILS_HPP
