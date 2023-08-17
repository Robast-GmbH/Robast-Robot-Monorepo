
#include "can/can_db.hpp"
#include "can_encoder_decoder.hpp"
#include "can_msgs/msg/frame.hpp"
#include "communication_interfaces/msg/drawer_task.hpp"
#include "communication_interfaces/msg/led_states.hpp"

namespace drawer_bridge
{
  class CanMessageCreator
  {
   public:
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using DrawerTask = communication_interfaces::msg::DrawerTask;
    using LedState = communication_interfaces::msg::LedState;
    using LedStates = communication_interfaces::msg::LedStates;

    can_msgs::msg::Frame create_can_msg_drawer_unlock(const DrawerAddress& msg) const;

    can_msgs::msg::Frame create_can_msg_drawer_task(const DrawerTask& msg) const;

    can_msgs::msg::Frame create_can_msg_led_header(const LedStates& msg) const;

    can_msgs::msg::Frame create_can_msg_set_single_led_state(const LedState& led_state,
                                                             const DrawerAddress& drawer_address) const;

   private:
    CanEncoderDecoder _can_encoder_decoder = CanEncoderDecoder();
    robast_can_msgs::CanDb _can_db = robast_can_msgs::CanDb();
  };
}   // namespace drawer_bridge
