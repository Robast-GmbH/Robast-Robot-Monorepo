#ifndef IDRAWER_HPP
#define IDRAWER_HPP

#include "can/can_message.h"

class IDrawer{
    public:
        virtual ~IDrawer(){};
        virtual void can_in(robast_can_msgs::CanMessage msg) = 0;
        virtual std::optional<robast_can_msgs::CanMessage> can_out() = 0;
        virtual void update_state() = 0;
};

#endif