#ifndef INTERFACES_I_DRAWER_HPP
#define INTERFACES_I_DRAWER_HPP

#include <optional>

#include "can/can_message.hpp"
#include "utils/e_drawer_task.hpp"

namespace interfaces
{
  class IDrawer
  {
   public:
    virtual ~IDrawer() = default;

    /**
     * Update all states of drawer.
     */
    virtual void update_state() = 0;

    /**
     * Unlock the drawer.
     */
    virtual void unlock() = 0;

    /**
     * Add a drawer task to the queue.
     *
     * @param e_drawer_task the task to be added to the queue
     */
    virtual void add_e_drawer_task_to_queue(const utils::EDrawerTask &e_drawer_task) = 0;

    /**
     * Set the state of the motor driver.
     *
     * @param enabled true if the motor driver should be enabled, false if it should be disabled
     * @param motor_id the id of the motor
     */
    virtual void set_motor_driver_state(const bool enabled, const uint8_t motor_id) const = 0;

   private:
    /**
     * Handles the procedure of what to do when drawer just opened.
     */
    virtual void handle_drawer_just_opened() = 0;

    /**
     * Handles the procedure of what to do when drawer just closed.
     */
    virtual void handle_drawer_just_closed() = 0;

    /**
     * Handle the control of the lock of the drawer.
     */
    virtual void handle_drawer_lock_control() = 0;
  };
}   // namespace interfaces

#endif   // INTERFACES_I_DRAWER_HPP