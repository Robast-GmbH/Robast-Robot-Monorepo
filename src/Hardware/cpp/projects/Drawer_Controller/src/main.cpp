#include <memory>

#include "drawer_controller/global.hpp"
#include "led/led_strip.hpp"

constexpr uint32_t MODULE_ID = 1;
constexpr uint8_t LOCK_ID = 0;
constexpr bool USE_ENCODER = false;

constexpr uint8_t NUM_OF_LEDS = 21;

std::unique_ptr<led::LedStrip<peripherals::pinout::LED_PIXEL_PIN, NUM_OF_LEDS>> led_strip;

stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
  .stepper_enn_tmc2209_pin_id = drawer_controller::pin_id::STEPPER_1_ENN_TMC2209,
  .stepper_stdby_tmc2209_pin_id = drawer_controller::pin_id::STEPPER_1_STDBY_TMC2209,
  .stepper_spread_pin_id = drawer_controller::pin_id::STEPPER_1_SPREAD,
  .stepper_dir_pin_id = drawer_controller::pin_id::STEPPER_1_DIR,
  .stepper_diag_pin_id = drawer_controller::pin_id::STEPPER_1_DIAG,
  .stepper_index_pin_id = drawer_controller::pin_id::STEPPER_1_INDEX,
  .stepper_step_pin_id = drawer_controller::pin_id::STEPPER_1_STEP};

/**********************************************************************************************************************
 * The program flow is organized as follows:
 * (1) setup() is called once at the beginning of the program
 * (2) The task loops contain a loop and run in parallel, whereby
 *    - receive_can_msg_task_loop() is responsible for receiving CAN messages and adding them to the queue
 *    - process_can_msgs_task_loop() is responsible for processing the CAN messages from the queue
 **********************************************************************************************************************/

using namespace drawer_controller;

void process_can_msgs_task_loop(void* pvParameters)
{
  for (;;)
  {
    std::optional<robast_can_msgs::CanMessage> received_message;
    if (xSemaphoreTake(can_queue_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
    {
      received_message = can_msg_queue->dequeue();
      xSemaphoreGive(can_queue_mutex);
    }
    else
    {
      Serial.println("Error: Could not take the mutex. This should not occur.");
    }

    if (received_message.has_value())
    {
      switch (received_message->get_id())
      {
        case robast_can_msgs::can_id::DRAWER_UNLOCK:
        {
          e_drawer->unlock();
        }
        break;
        case robast_can_msgs::can_id::ELECTRICAL_DRAWER_TASK:
        {
          utils::EDrawerTask e_drawer_task = can_message_converter->convert_to_e_drawer_task(received_message.value());
          e_drawer->add_e_drawer_task_to_queue(e_drawer_task);
        }
        break;
        case robast_can_msgs::can_id::LED_HEADER:
        {
          led::LedHeader led_header = can_message_converter->convert_to_led_header(received_message.value());
          led_strip->initialize_led_state_change(led_header);
        }
        break;
        case robast_can_msgs::can_id::SINGLE_LED_STATE:
        {
          led::LedState led_state = can_message_converter->convert_to_led_state(received_message.value());
          led_strip->set_led_state(led_state);
        }
        break;
        case robast_can_msgs::can_id::MODULE_CONFIG:
        {
          bool config_set_successfully =
            config_manager->set_config(received_message->get_can_signals()
                                         .at(robast_can_msgs::can_signal::id::module_config::CONFIG_ID)
                                         .get_data(),
                                       received_message->get_can_signals()
                                         .at(robast_can_msgs::can_signal::id::module_config::CONFIG_VALUE)
                                         .get_data());
          if (!config_set_successfully)
          {
            Serial.println("[Main]: Warning - Tried to set config for invalid config id!");
          }
        }
        default:
          debug_println("[Main]: Received unsupported CAN message.");
          break;
      }
    }

    led_strip->handle_led_control();

    e_drawer->update_state();

    std::optional<robast_can_msgs::CanMessage> to_be_sent_message = e_drawer->can_out();

    if (to_be_sent_message.has_value())
    {
      can_controller->send_can_message(to_be_sent_message.value());
    }
  }
}

void setup()
{
  debug_setup(115200);
  debug_println("[Main]: Start...");

  std::shared_ptr<TwoWire> wire_port_expander = std::make_shared<TwoWire>(2);
  wire_port_expander->begin(peripherals::i2c::I2C_SDA, peripherals::i2c::I2C_SCL);

  gpio_wrapper = std::make_shared<gpio::GpioWrapperPca9535>(
    wire_port_expander, slave_address_to_port_expander, pin_mapping_id_to_gpio_info, pin_mapping_id_to_port);

  endstop_switch = std::make_shared<switch_lib::Switch>(gpio_wrapper,
                                                        pin_id::SENSE_INPUT_DRAWER_1_CLOSED,
                                                        SWITCH_PRESSED_THRESHOLD,
                                                        switch_lib::Switch::normally_closed,
                                                        SWITCH_WEIGHT_NEW_VALUES);

  can_db = std::make_shared<robast_can_msgs::CanDb>();
  can_message_converter = std::make_unique<utils::CanMessageConverter>();

  led_strip = std::make_unique<led::LedStrip<peripherals::pinout::LED_PIXEL_PIN, NUM_OF_LEDS>>();

  can_controller = std::make_unique<can_toolbox::CanController>(
    MODULE_ID, can_db, peripherals::pinout::TWAI_TX_PIN, peripherals::pinout::TWAI_RX_PIN);

  can_queue_mutex = xSemaphoreCreateMutex();
  can_msg_queue = std::make_unique<utils::Queue<robast_can_msgs::CanMessage>>();

  drawer_lock = std::make_shared<lock::ElectricalDrawerLock>(gpio_wrapper,
                                                             pin_id::LOCK_1_OPEN_CONTROL,
                                                             pin_id::LOCK_1_CLOSE_CONTROL,
                                                             pin_id::LOCK_1_SENSE,
                                                             SWITCH_PRESSED_THRESHOLD,
                                                             SWITCH_WEIGHT_NEW_VALUES);

  drawer_config = std::make_shared<drawer::ElectricalDrawerConfig>();
  encoder_config = std::make_shared<motor::EncoderConfig>();
  motor_config = std::make_shared<motor::MotorConfig>();
  motor_monitor_config = std::make_shared<motor::MotorMonitorConfig>();

  config_manager =
    std::make_unique<utils::ConfigManager>(drawer_config, encoder_config, motor_config, motor_monitor_config);

  e_drawer =
    std::make_shared<drawer::ElectricalDrawer>(MODULE_ID,
                                               LOCK_ID,
                                               can_db,
                                               gpio_wrapper,
                                               stepper_1_pin_id_config,
                                               USE_ENCODER,
                                               gpio_wrapper->get_gpio_num_for_pin_id(pin_id::STEPPER_1_ENCODER_A),
                                               gpio_wrapper->get_gpio_num_for_pin_id(pin_id::STEPPER_1_ENCODER_B),
                                               STEPPER_MOTOR_1_ADDRESS,
                                               motor_config,
                                               endstop_switch,
                                               drawer_lock,
                                               drawer_config,
                                               encoder_config,
                                               motor_monitor_config);

  debug_println("[Main]: Finished setup()!");

  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(receive_can_msg_task_loop, /* Task function. */
                          "Task1",                   /* name of task. */
                          10000,                     /* Stack size of task */
                          NULL,                      /* parameter of the task */
                          1,                         /* priority of the task */
                          &Task1,                    /* Task handle to keep track of created task */
                          0);                        /* pin task to core 0 */

  delay(100);

  xTaskCreatePinnedToCore(process_can_msgs_task_loop, /* Task function. */
                          "Task2",                    /* name of task. */
                          20000,                      /* Stack size of task */
                          NULL,                       /* parameter of the task */
                          1,                          /* priority of the task */
                          &Task2,                     /* Task handle to keep track of created task */
                          1);                         /* pin task to core 1 */
}

void loop()
{
}
