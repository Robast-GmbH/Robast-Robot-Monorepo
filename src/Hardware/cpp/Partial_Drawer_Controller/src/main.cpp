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

std::unique_ptr<drawer_controller::DataMapper> data_mapper;

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
  data_mapper = std::make_unique<drawer_controller::DataMapper>();

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
  for (uint8_t i = 1; i <= num_of_can_readings_per_cycle; ++i)
  {
    std::optional<robast_can_msgs::CanMessage> received_message = can_controller->handle_receiving_can_msg();

    if (received_message.has_value())
    {
      switch (received_message->get_id())
      {
        case CAN_ID_DRAWER_UNLOCK:
        {
          uint8_t tray_id = received_message->get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data();
          tray_manager->unlock_lock(tray_id);
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
        case CAN_ID_TRAY_LED_BRIGHTNESS:
        {
          tray_manager->set_tray_led_brightness(
            received_message->get_can_signals().at(CAN_SIGNAL_TRAY_ID).get_data(),
            received_message->get_can_signals().at(CAN_SIGNAL_TRAY_LED_ROW_INDEX).get_data(),
            received_message->get_can_signals().at(CAN_SIGNAL_TRAY_LED_STATE_BRIGHNESS).get_data());
        }
        default:
          debug_println("Received unsupported CAN message.");
          break;
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

  tray_manager->update_states();
}
