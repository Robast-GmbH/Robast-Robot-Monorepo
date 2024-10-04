
#include "can/can_db.hpp"
#include "can_encoder_decoder.hpp"
#include "can_msgs/msg/frame.hpp"
#include "communication_interfaces/msg/drawer_task.hpp"
#include "communication_interfaces/msg/led.hpp"
#include "communication_interfaces/msg/led_cmd.hpp"
#include "communication_interfaces/msg/tray_task.hpp"
#include "communication_interfaces/srv/electrical_drawer_motor_control.hpp"

namespace drawer_bridge
{
  class CanMessageCreator
  {
   public:
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using DrawerTask = communication_interfaces::msg::DrawerTask;
    using Led = communication_interfaces::msg::Led;
    using LedCmd = communication_interfaces::msg::LedCmd;
    using TrayTask = communication_interfaces::msg::TrayTask;

    using ElectricalDrawerMotorControl = communication_interfaces::srv::ElectricalDrawerMotorControl;

    can_msgs::msg::Frame create_can_msg_drawer_unlock(const DrawerAddress& msg) const;

    can_msgs::msg::Frame create_can_msg_drawer_task(const DrawerTask& msg) const;

    can_msgs::msg::Frame create_can_msg_led_header(const LedCmd& msg) const;

    can_msgs::msg::Frame create_can_msg_set_single_led_state(const Led& led_state,
                                                             const DrawerAddress& drawer_address) const;

    can_msgs::msg::Frame create_can_msg_tray_led_brightness(const DrawerAddress& drawer_address,
                                                            const uint8_t led_row,
                                                            const uint8_t brightness) const;

    can_msgs::msg::Frame create_can_msg_set_module_config(const DrawerAddress& drawer_address,
                                                          const uint8_t config_id,
                                                          const uint32_t config_value) const;

    can_msgs::msg::Frame create_can_msg_e_drawer_motor_control(
      const std::shared_ptr<ElectricalDrawerMotorControl::Request> motor_control_request) const;

   private:
    CanEncoderDecoder _can_encoder_decoder = CanEncoderDecoder();
    robast_can_msgs::CanDb _can_db = robast_can_msgs::CanDb();
  };
}   // namespace drawer_bridge
