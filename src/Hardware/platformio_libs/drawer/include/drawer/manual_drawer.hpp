#ifndef DRAWER_MANUAL_DRAWER_HPP
#define DRAWER_MANUAL_DRAWER_HPP

#include "can_toolbox/can_utils.hpp"
#include "interfaces/i_drawer.hpp"
#include "lock/electrical_drawer_lock.hpp"
#include "switch/switch.hpp"

namespace drawer
{
  class ManualDrawer : public interfaces::IDrawer
  {
   public:
    ManualDrawer(const uint32_t module_id,
                 const uint8_t id,
                 const std::shared_ptr<can_toolbox::CanUtils> can_utils,
                 const std::shared_ptr<switch_lib::Switch> endstop_switch,
                 const std::shared_ptr<lock::ElectricalDrawerLock> drawer_lock);

    void update_state() override;

    void unlock() override;

    void add_e_drawer_task_to_queue(const utils::EDrawerTask &e_drawer_task) override;

    void set_motor_driver_state(const bool enabled, const uint8_t motor_id) const;

   private:
    const uint32_t _module_id;
    const uint8_t _id;

    const std::shared_ptr<switch_lib::Switch> _endstop_switch;

    const std::shared_ptr<lock::ElectricalDrawerLock> _drawer_lock;

    const std::shared_ptr<can_toolbox::CanUtils> _can_utils;

    bool _triggered_closing_lock_after_opening = false;

    void handle_drawer_lock_control() override;

    void handle_drawer_just_opened() override;

    void handle_drawer_just_closed() override;
  };

}   // namespace drawer

#endif   // DRAWER_MANUAL_DRAWER_HPP