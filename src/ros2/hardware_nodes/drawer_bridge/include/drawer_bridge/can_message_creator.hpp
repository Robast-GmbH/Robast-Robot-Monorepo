
#include "can/can_db.hpp"
#include "can_encoder_decoder.hpp"
#include "can_msgs/msg/frame.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"
#include "communication_interfaces/msg/drawer_task.hpp"

namespace drawer_bridge
{
  class CanMessageCreator
  {
   public:
    using CanMessage = can_msgs::msg::Frame;
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using DrawerTask = communication_interfaces::msg::DrawerTask;
    using DrawerLeds = communication_interfaces::msg::DrawerLeds;

    CanMessage create_can_msg_drawer_unlock(const DrawerAddress& msg) const;

    CanMessage create_can_msg_drawer_led(const DrawerLeds& msg) const;

    CanMessage create_can_msg_drawer_task(const DrawerTask& msg) const;

   private:
    CanEncoderDecoder _can_encoder_decoder = CanEncoderDecoder();
    robast_can_msgs::CanDb _can_db = robast_can_msgs::CanDb();
  };
}   // namespace drawer_bridge
