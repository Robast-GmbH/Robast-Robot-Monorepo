
#include "can_msgs/msg/frame.hpp"
#include "communication_interfaces/msg/drawer.hpp"
#include "communication_interfaces/msg/drawer_task.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"
#include "can_encoder_decoder.hpp"
#include "can/can_db.hpp"

class CanMessageCreator {
public:
    using CanMessage = can_msgs::msg::Frame;
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using DrawerTask = communication_interfaces::msg::DrawerTask;
    using DrawerLeds = communication_interfaces::msg::DrawerLeds;
    

    CanMessage create_can_msg_drawer_lock(const DrawerAddress& msg,
      uint8_t can_data_open_lock) const;

    CanMessage create_can_msg_drawer_led(const DrawerLeds& msg) const;

    CanMessage create_can_msg_drawer_task(const DrawerTask& msg)const;

private:
    CanEncoderDecoder can_encoder_decoder_ = CanEncoderDecoder();
    robast_can_msgs::CanDb can_db_ = robast_can_msgs::CanDb();
};