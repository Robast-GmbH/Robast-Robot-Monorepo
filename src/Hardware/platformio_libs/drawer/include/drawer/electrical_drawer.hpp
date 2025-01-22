#ifndef DRAWER_ELECTRICAL_DRAWER_HPP
#define DRAWER_ELECTRICAL_DRAWER_HPP

#include <Arduino.h>

#include <memory>
#include <optional>

#include "can_toolbox/can_utils.hpp"
#include "drawer/electrical_drawer_config.hpp"
#include "drawer/motion_controller.hpp"
#include "interfaces/i_drawer.hpp"
#include "lock/electrical_drawer_lock.hpp"
#include "switch/switch.hpp"
#include "utils/e_drawer_task.hpp"

namespace drawer
{
  constexpr bool IS_HOMING = true;
  constexpr bool IS_NOT_HOMING = false;
  constexpr bool DO_NOT_USE_ACCELERATION_RAMP = false;
  constexpr bool PUSH_TO_CLOSE_TRIGGERED = true;

  constexpr bool ENDSTOP_SWITCH_IS_PUSHED = true;

  class ElectricalDrawer : public interfaces::IDrawer
  {
   public:
    ElectricalDrawer(const uint32_t module_id,
                     const uint8_t id,
                     const std::shared_ptr<can_toolbox::CanUtils> can_utils,
                     const std::shared_ptr<switch_lib::Switch> endstop_switch,
                     const std::shared_ptr<ElectricalDrawerConfig> e_drawer_config,
                     const std::shared_ptr<drawer::MotionController> motion_controller,
                     const std::optional<std::shared_ptr<lock::ElectricalDrawerLock>> drawer_lock);

    void update_state() override;

    void unlock() override;

    void add_e_drawer_task_to_queue(const utils::EDrawerTask &e_drawer_task) override;

   private:
    const uint32_t _module_id;
    const uint8_t _id;

    const std::shared_ptr<switch_lib::Switch> _endstop_switch;

    // optional because the lock is not always installed (e.g. in the partial drawer)
    const std::optional<std::shared_ptr<lock::ElectricalDrawerLock>> _drawer_lock;

    const std::shared_ptr<can_toolbox::CanUtils> _can_utils;

    std::unique_ptr<utils::Queue<utils::EDrawerTask>> _e_drawer_task_queue =
      std::make_unique<utils::Queue<utils::EDrawerTask>>();

    const std::shared_ptr<drawer::ElectricalDrawerConfig> _config;

    const std::shared_ptr<drawer::MotionController> _motion_controller;

    bool _triggered_closing_lock_after_opening = false;

    uint32_t _timestamp_drawer_opened_in_ms = 0;

    /* FUNCTIONS */

    void handle_drawer_idle_state();

    void handle_drawer_active_state();

    void start_next_e_drawer_task();

    void check_if_drawer_is_pulled_out();

    void handle_drawer_just_opened() override;

    void handle_drawer_just_closed() override;

    void handle_drawer_lock_control() override;

    void handle_drawer_moving_in();

    void handle_drawer_moving_out();

    void handle_push_to_close_triggered();

    void debug_prints_moving_e_drawer();

    bool is_drawer_closed() const;

    void check_for_drawer_not_opened_error() const;
  };
}   // namespace drawer

#endif   // DRAWER_ELECTRICAL_DRAWER_HPP
