#include <memory>

#include "can.hpp"
#include "drawer.hpp"
#include "electrical_drawer.hpp"
#include "gpio.hpp"
#include "i_gpio_wrapper.hpp"
#include "led_strip.hpp"

#define MODULE_ID               1
#define LOCK_ID                 0
#define STEPPER_MOTOR_1_ADDRESS 0

using drawer_ptr = std::shared_ptr<drawer_controller::IDrawer>;

std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper = std::make_shared<drawer_controller::GPIO>();

stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
    .stepper_en_tmc2209_pin_id = STEPPER_1_EN_TMC2209_PIN_ID,
    .stepper_stdby_tmc2209_pin_id = STEPPER_1_STDBY_TMC2209_PIN_ID,
    .stepper_spread_pin_id = STEPPER_1_SPREAD_PIN_ID,
    .stepper_dir_pin_id = STEPPER_1_DIR_PIN_ID,
    .stepper_diag_pin_id = STEPPER_1_DIAG_PIN_ID,
    .stepper_index_pin_id = STEPPER_1_INDEX_PIN_ID,
    .stepper_step_pin_id = STEPPER_1_STEP_PIN_ID};

drawer_controller::ElectricalDrawer e_drawer_0 = drawer_controller::ElectricalDrawer(
    MODULE_ID, LOCK_ID, gpio_wrapper, stepper_1_pin_id_config, DRAWER_1_ENCODER_A_PIN, DRAWER_1_ENCODER_B_PIN);

drawer_controller::Drawer drawer_0 = drawer_controller::Drawer(MODULE_ID, LOCK_ID, gpio_wrapper);
std::vector<drawer_ptr> drawers = std::vector<drawer_ptr>();
drawer_controller::Can CAN = drawer_controller::Can(MODULE_ID, gpio_wrapper, OE_TXB0104_PIN_ID, PCA9554_OUTPUT);

std::optional<robast_can_msgs::CanMessage> received_message;
std::optional<robast_can_msgs::CanMessage> to_be_sent_message;

void setup()
{
  Serial.begin(115200);   // Init serial port and set baudrate
  while (!Serial)
  {
  }
  Serial.println("\nStart...");
  drawer_0.init_lock(LOCK_1_OPEN_CONROL_PIN_ID,
                     LOCK_1_CLOSE_CONROL_PIN_ID,
                     SENSE_INPUT_LOCK_1_PIN_ID,
                     SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID,
                     PCA9554_INPUT,
                     PCA9554_OUTPUT);
  drawers.push_back(std::make_shared<drawer_controller::Drawer>(drawer_0));
  led_strip::initialize_led_strip();
  CAN.initialize_can_controller();

  e_drawer_0.init_motor(STEPPER_MOTOR_1_ADDRESS);
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
        case CAN_ID_DRAWER_UNLOCK:
        {
          uint8_t drawer_id = received_message->get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data();
          drawers.at(drawer_id)->can_in(received_message.value());
        }
        break;
        case CAN_ID_ELECTRICAL_DRAWER_TASK:
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
  if (Serial.available())
  {
    char input = Serial.read();
    if (input == '-')
    {
      Serial.println("stop motor");
      e_drawer_0.stop_motor();
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
