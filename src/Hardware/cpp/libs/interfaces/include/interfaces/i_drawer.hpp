#ifndef INTERFACES_I_DRAWER_HPP
#define INTERFACES_I_DRAWER_HPP

#include <optional>

#include "can/can_message.hpp"

namespace interfaces
{
  class IDrawer
  {
   public:
    virtual ~IDrawer() {};

    /**
     * Returns CAN messages of the drawer that should be sent via the CAN bus.
     *
     * @return None, if no CAN message is available. Otherwise returns a CAN message.
     */
    virtual std::optional<robast_can_msgs::CanMessage> can_out() = 0;

    /**
     * Update all states of drawer.
     */
    virtual void update_state() = 0;

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
    virtual void handle_e_drawer_lock_control() = 0;
  };
}   // namespace interfaces

#endif  // INTERFACES_I_DRAWER_HPP