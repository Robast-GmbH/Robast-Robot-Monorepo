#include <memory>

#include "can.hpp"
#include "drawer.hpp"
#include "gpio.hpp"
#include "led_strip.hpp"

#define MODULE_ID 1

using drawer_ptr = std::shared_ptr<drawer_controller::IDrawer>;

drawer_controller::Drawer drawer_0 = drawer_controller::Drawer(MODULE_ID, 0);
std::vector<drawer_ptr> drawers = std::vector<drawer_ptr>();
drawer_controller::Can CAN = drawer_controller::Can(MODULE_ID);

std::optional<robast_can_msgs::CanMessage> received_message;
std::optional<robast_can_msgs::CanMessage> to_be_sent_message;

void setup()
{
  Serial.begin(115200);   // Init serial port and set baudrate
  while (!Serial)
  {
  }
  Serial.println("\nStart...");
  drawer_0.init_lock(LOCK_1_OPEN_CONROL_ID,
                     LOCK_1_CLOSE_CONROL_ID,
                     SENSE_INPUT_LOCK_1_ID,
                     SENSE_INPUT_DRAWER_1_CLOSED_ID,
                     std::make_unique<drawer_controller::GPIO>());
  drawers.push_back(std::make_shared<drawer_controller::Drawer>(drawer_0));
  led_strip::initialize_led_strip();
  CAN.initialize_can_controller();
}

void loop()
{
  if (CAN.is_message_available())
  {
    received_message = CAN.handle_receiving_can_msg();

    if (received_message.has_value())
    {
      switch (received_message->get_id())
      {
        case CAN_ID_DRAWER_UNLOCK || CAN_ID_ELECTRICAL_DRAWER_TASK:
        {
          uint8_t drawer_id = received_message->get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data();
          drawers.at(drawer_id)->can_in(received_message.value());
        }
        break;
        case CAN_ID_DRAWER_LED:
          led_strip::add_led_strip_mode_to_queue(*received_message);
          led_strip::debug_prints_drawer_led(*received_message);
          break;
        default:
          Serial.println("Received unsupported CAN message.");
          break;
      }
    }
  }

  led_strip::handle_led_control();

  for (drawer_ptr drawer : drawers)
  {
    drawer->update_state();
    to_be_sent_message = drawer->can_out();
    if (to_be_sent_message.has_value())
    {
      CAN.send_can_message(to_be_sent_message.value());
    }
  }
}