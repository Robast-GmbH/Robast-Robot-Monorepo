#ifndef DRAWER_CONTROLLER_IDRAWER_HPP
#define DRAWER_CONTROLLER_IDRAWER_HPP

#include "can/can_message.h"

namespace drawer_controller
{
  class IDrawer
  {
   public:
    virtual ~IDrawer(){};
    virtual void can_in(robast_can_msgs::CanMessage msg) = 0;
    virtual std::optional<robast_can_msgs::CanMessage> can_out() = 0;
    virtual void update_state() = 0;
    virtual void handle_drawer_just_opened() = 0;
    virtual void handle_drawer_just_closed() = 0;
  };
}   // namespace drawer_controller
#endif