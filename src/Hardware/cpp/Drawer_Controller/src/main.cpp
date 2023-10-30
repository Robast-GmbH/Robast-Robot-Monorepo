#include <memory>

#include "can/can_controller.hpp"
#include "debug/debug.hpp"
#include "drawer/drawer.hpp"
#include "drawer/electrical_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "led/led_strip.hpp"
#include "peripherals/gpio.hpp"
#include "utils/data_mapper.hpp"

#define MODULE_ID               1
#define LOCK_ID                 0
#define STEPPER_MOTOR_1_ADDRESS 0
#define USE_ENCODER             0

using drawer_ptr = std::shared_ptr<drawer_controller::IDrawer>;

std::shared_ptr<robast_can_msgs::CanDb> can_db;

std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper;

std::shared_ptr<drawer_controller::ElectricalDrawer> e_drawer_0;

stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
  .stepper_enn_tmc2209_pin_id = STEPPER_1_ENN_TMC2209_PIN_ID,
  .stepper_stdby_tmc2209_pin_id = STEPPER_1_STDBY_TMC2209_PIN_ID,
  .stepper_spread_pin_id = STEPPER_1_SPREAD_PIN_ID,
  .stepper_dir_pin_id = STEPPER_1_DIR_PIN_ID,
  .stepper_diag_pin_id = STEPPER_1_DIAG_PIN_ID,
  .stepper_index_pin_id = STEPPER_1_INDEX_PIN_ID,
  .stepper_step_pin_id = STEPPER_1_STEP_PIN_ID};

// TODO@all: If you want to use a "normal" drawer uncomment this and comment out the e_drawer
// TODO@Andres: Write a script to automatically generate code with a drawer / e-drawer
std::shared_ptr<drawer_controller::Drawer> drawer_0;

std::vector<drawer_ptr> drawers = std::vector<drawer_ptr>();

std::unique_ptr<drawer_controller::CanController> can_controller;

std::unique_ptr<drawer_controller::LedStrip> led_strip;

std::unique_ptr<drawer_controller::DataMapper> data_mapper;

// TODO@Jacob: This is kind of a temporary hack to ensure that the can messages are read from the controller fast enough
// TODO@Jacob: A much better solution would be to create paralle tasks for that like I suggested in this story:
// https://robast.atlassian.net/browse/RE-1775
uint16_t num_of_can_readings_per_cycle = 1;

void setup()
{
  debug_setup(115200);
  debug_println("\nStart...");

  can_db = std::make_shared<robast_can_msgs::CanDb>();
  gpio_wrapper = std::make_shared<drawer_controller::GPIO>();
  data_mapper = std::make_unique<drawer_controller::DataMapper>();

  // TODO@all: If you want to use a "normal" drawer uncomment this and comment out the e_drawer
  // TODO@Andres: Write a script to automatically generate code with a drawer / e-drawer
  drawer_0 = std::make_shared<drawer_controller::Drawer>(MODULE_ID, LOCK_ID, can_db, gpio_wrapper);
  drawer_0->init_electrical_lock(LOCK_1_OPEN_CONROL_PIN_ID,
                                 LOCK_1_CLOSE_CONROL_PIN_ID,
                                 SENSE_INPUT_LOCK_1_PIN_ID,
                                 SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID);
  drawers.push_back(drawer_0);

  led_strip = std::make_unique<drawer_controller::LedStrip>();

  can_controller = std::make_unique<drawer_controller::CanController>(
    MODULE_ID, can_db, gpio_wrapper, OE_TXB0104_PIN_ID, PCA9554_OUTPUT);
  can_controller->initialize_can_controller();

  // e_drawer_0 = std::make_shared<drawer_controller::ElectricalDrawer>(MODULE_ID,
  //                                                                    LOCK_ID,
  //                                                                    can_db,
  //                                                                    gpio_wrapper,
  //                                                                    stepper_1_pin_id_config,
  //                                                                    USE_ENCODER,
  //                                                                    DRAWER_1_ENCODER_A_PIN,
  //                                                                    DRAWER_1_ENCODER_B_PIN,
  //                                                                    STEPPER_MOTOR_1_ADDRESS);
  // e_drawer_0->init_electrical_lock(LOCK_1_OPEN_CONROL_PIN_ID,
  //                                  LOCK_1_CLOSE_CONROL_PIN_ID,
  //                                  SENSE_INPUT_LOCK_1_PIN_ID,
  //                                  SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID);
  // e_drawer_0->init_motor();
  // drawers.push_back(e_drawer_0);

  debug_println("Finished setup()!");
}

void loop()
{
  for (uint8_t i = 1; i <= num_of_can_readings_per_cycle; ++i)
  {
    if (can_controller->is_message_available())
    {
      std::optional<robast_can_msgs::CanMessage> received_message = can_controller->handle_receiving_can_msg();

      if (received_message.has_value())
      {
        switch (received_message->get_id())
        {
          case CAN_ID_DRAWER_UNLOCK:
          {
            uint8_t drawer_id = received_message->get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data();
            // TODO@Jacob: remove can_in function and put this into the data_mapper to remove can dependency from drawer
            drawers.at(drawer_id)->can_in(received_message.value());
          }
          break;
          case CAN_ID_ELECTRICAL_DRAWER_TASK:
          {
            uint8_t drawer_id = received_message->get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data();
            // TODO@Jacob: remove can_in function and put this into the data_mapper to remove can dependency from drawer
            drawers.at(drawer_id)->can_in(received_message.value());
          }
          break;
          case CAN_ID_LED_HEADER:
          {
            drawer_controller::LedHeader led_header = data_mapper->create_led_header(received_message.value());
            num_of_can_readings_per_cycle = led_header.num_of_led_states_to_change + LED_HEADER_CAN_MSG_COUNT;
            led_strip->initialize_led_state_change(led_header);
          }
          break;
          case CAN_ID_SINGLE_LED_STATE:
          {
            drawer_controller::LedState led_state = data_mapper->create_led_state(received_message.value());
            led_strip->set_led_state(led_state);
          }
          break;
          default:
            debug_println("Received unsupported CAN message.");
            break;
        }
      }
    }
  }
  num_of_can_readings_per_cycle = 1;

  led_strip->handle_led_control();

  for (drawer_ptr drawer : drawers)
  {
    drawer->update_state();

    std::optional<robast_can_msgs::CanMessage> to_be_sent_message = drawer->can_out();

    if (to_be_sent_message.has_value())
    {
      can_controller->send_can_message(to_be_sent_message.value());
    }
  }
}
