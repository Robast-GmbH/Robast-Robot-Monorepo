#include <memory>

#include "can/can_controller.hpp"
#include "debug/debug.hpp"
#include "drawer/electrical_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "led/led_strip.hpp"
#include "lock/tray_manager.hpp"
#include "peripherals/gpio.hpp"
#include "utils/data_mapper.hpp"

#define MODULE_ID               1
#define LOCK_ID                 0
#define STEPPER_MOTOR_1_ADDRESS 0
#define USE_ENCODER             0

using drawer_ptr = std::shared_ptr<drawer_controller::IDrawer>;

std::shared_ptr<robast_can_msgs::CanDb> can_db;

std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper;

std::vector<drawer_ptr> drawers = std::vector<drawer_ptr>();

std::shared_ptr<drawer_controller::ElectricalDrawer> e_drawer_0;

stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
  .stepper_enn_tmc2209_pin_id = STEPPER_1_ENN_TMC2209_PIN_ID,
  .stepper_stdby_tmc2209_pin_id = STEPPER_1_STDBY_TMC2209_PIN_ID,
  .stepper_spread_pin_id = STEPPER_1_SPREAD_PIN_ID,
  .stepper_dir_pin_id = STEPPER_1_DIR_PIN_ID,
  .stepper_diag_pin_id = STEPPER_1_DIAG_PIN_ID,
  .stepper_index_pin_id = STEPPER_1_INDEX_PIN_ID,
  .stepper_step_pin_id = STEPPER_1_STEP_PIN_ID};

std::shared_ptr<partial_drawer_controller::TrayManager> tray_manager;

std::unique_ptr<drawer_controller::LedStrip> led_strip;

// std::unique_ptr<drawer_controller::DataMapper> data_mapper;

std::unique_ptr<drawer_controller::CanController> can_controller;

// TODO@Jacob: This is kind of a temporary hack to ensure that the can messages are read from the
// controller fast enough
// TODO@Jacob: A much better solution would be to create paralle tasks for that like I suggested in
// this story: https://robast.atlassian.net/browse/RE-1775
uint16_t num_of_can_readings_per_cycle = 1;

// TODO: REMOVE THIS
uint16_t counter = 0;
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
unsigned long previousMillis = 0;   // will store last time a CAN Message was send

void setup()
{
  debug_setup(115200);
  debug_println("\nStart...");

  // TODO: In the hardware design of v1 the pins of SDA and SCL are mixed up unfortunately for the port expander
  // TODO: In order to use the hardware anyway, we need to create two instances of the bus with different pin init
  std::shared_ptr<TwoWire> wire_onboard_led_driver = std::make_shared<TwoWire>(1);
  std::shared_ptr<TwoWire> wire_port_expander = std::make_shared<TwoWire>(2);
  wire_onboard_led_driver->begin(I2C_SDA, I2C_SCL);
  wire_port_expander->begin(I2C_SDA_PORT_EXPANDER, I2C_SCL_PORT_EXPANDER);

  gpio_wrapper = std::make_shared<drawer_controller::GPIO>(wire_port_expander);

  std::vector<partial_drawer_controller::TrayPinConfig> tray_pin_configs = {
    {LOCK_1_OPEN_CONTROL_PIN_ID, LOCK_1_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_1_CLOSED_PIN_ID},
    {LOCK_2_OPEN_CONTROL_PIN_ID, LOCK_2_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_2_CLOSED_PIN_ID},
    {LOCK_3_OPEN_CONTROL_PIN_ID, LOCK_3_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_3_CLOSED_PIN_ID},
    {LOCK_4_OPEN_CONTROL_PIN_ID, LOCK_4_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_4_CLOSED_PIN_ID},
    {LOCK_5_OPEN_CONTROL_PIN_ID, LOCK_5_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_5_CLOSED_PIN_ID},
    {LOCK_6_OPEN_CONTROL_PIN_ID, LOCK_6_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_6_CLOSED_PIN_ID},
    {LOCK_7_OPEN_CONTROL_PIN_ID, LOCK_7_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_7_CLOSED_PIN_ID},
    {LOCK_8_OPEN_CONTROL_PIN_ID, LOCK_8_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_8_CLOSED_PIN_ID}};

  tray_manager =
    std::make_shared<partial_drawer_controller::TrayManager>(tray_pin_configs, gpio_wrapper, wire_onboard_led_driver);

  auto set_led_driver_enable_pin_high = []()
  {
    gpio_wrapper->set_pin_mode(ENABLE_ONBOARD_LED_VDD_PIN_ID, gpio_wrapper->get_gpio_output_pin_mode());
    gpio_wrapper->digital_write(ENABLE_ONBOARD_LED_VDD_PIN_ID, true);
  };
  tray_manager->init(set_led_driver_enable_pin_high);

  can_db = std::make_shared<robast_can_msgs::CanDb>();
  // data_mapper = std::make_unique<drawer_controller::DataMapper>();

  led_strip = std::make_unique<drawer_controller::LedStrip>();

  can_controller = std::make_unique<drawer_controller::CanController>(MODULE_ID, can_db, gpio_wrapper);
  can_controller->initialize_can_controller();

  e_drawer_0 = std::make_shared<drawer_controller::ElectricalDrawer>(MODULE_ID,
                                                                     LOCK_ID,
                                                                     can_db,
                                                                     gpio_wrapper,
                                                                     stepper_1_pin_id_config,
                                                                     USE_ENCODER,
                                                                     DRAWER_1_ENCODER_A_PIN,
                                                                     DRAWER_1_ENCODER_B_PIN,
                                                                     STEPPER_MOTOR_1_ADDRESS,
                                                                     SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID,
                                                                     std::nullopt);
  e_drawer_0->init();

  drawers.push_back(e_drawer_0);

  debug_println("Finished setup()!");
}

void loop()
{
  std::optional<robast_can_msgs::CanMessage> received_message = can_controller->handle_receiving_can_msg();

  if (received_message.has_value())
  {
    debug_println("Main.cpp: Received CAN message!");
  }

  // led_strip->handle_led_control();

  // uint8_t value;
  // gpio_wrapper->digital_read(SENSE_INPUT_LID_7_CLOSED_PIN_ID, value);

  // if (value == 1)
  // {
  //   if (counter == 400)
  //   {
  //     debug_println("Lid 7 is open!");
  //     counter = 0;
  //   }
  // }
  // else
  // {
  //   if (counter == 400)
  //   {
  //     debug_println("Lid 7 is closed!");
  //     counter = 0;
  //   }
  // }
  // counter++;

  // if keyboard input toggle lock
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == 'o')
    {
      debug_println("Open lock 8");
      gpio_wrapper->digital_write(LOCK_7_CLOSE_CONTROL_PIN_ID, false);
      delay(100);
      gpio_wrapper->digital_write(LOCK_7_OPEN_CONTROL_PIN_ID, true);
    }
    else if (c == 'c')
    {
      debug_println("Close lock 8");
      gpio_wrapper->digital_write(LOCK_7_OPEN_CONTROL_PIN_ID, false);
      delay(100);
      gpio_wrapper->digital_write(LOCK_7_CLOSE_CONTROL_PIN_ID, true);
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    debug_println("Sending CAN message!");
    robast_can_msgs::CanMessage can_msg_error_feedback = can_db->can_messages.at(CAN_MSG_ERROR_FEEDBACK);
    std::vector can_signals_error_feedback = can_msg_error_feedback.get_can_signals();

    can_signals_error_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(1);
    can_signals_error_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(2);
    can_signals_error_feedback.at(CAN_SIGNAL_ERROR_CODE).set_data(3);

    can_msg_error_feedback.set_can_signals(can_signals_error_feedback);

    can_controller->send_can_message(can_msg_error_feedback);
  }
}
