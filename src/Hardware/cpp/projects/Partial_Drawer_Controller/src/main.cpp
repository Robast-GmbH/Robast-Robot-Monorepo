#include <memory>

#include "drawer_controller/global.hpp"
#include "lock/tray_manager.hpp"

constexpr uint32_t MODULE_ID = 6;
constexpr uint8_t LOCK_ID = 0;
constexpr bool USE_ENCODER = true;

constexpr uint8_t NUM_OF_LEDS = 21;

std::unique_ptr<led::LedStrip<peripherals::pinout::LED_PIXEL_PIN, NUM_OF_LEDS>> led_strip;

std::shared_ptr<partial_drawer_controller::TrayManager> tray_manager;

/**********************************************************************************************************************
 * The program flow is organized as follows:
 * (1) setup() is called once at the beginning of the program
 * (2) The task loops contain a loop and run in parallel, whereby
 *    - receive_can_msg_task_loop() is responsible for receiving CAN messages and adding them to the queue
 *    - process_can_msgs_task_loop() is responsible for processing the CAN messages from the queue
 **********************************************************************************************************************/

using namespace partial_drawer_controller;

void process_can_msgs_task_loop(void* pvParameters)
{
  global_params::TaskParams* task_params = static_cast<global_params::TaskParams*>(pvParameters);

  for (;;)
  {
    std::optional<robast_can_msgs::CanMessage> received_message;
    if (xSemaphoreTake(can_queue_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
    {
      received_message = task_params->can_msg_queue->dequeue();
      xSemaphoreGive(can_queue_mutex);
    }
    else
    {
      Serial.println("[Main]: Error: Could not take the mutex. This should not occur.");
    }
    if (received_message.has_value())
    {
      switch (received_message->get_id())
      {
        case robast_can_msgs::can_id::DRAWER_UNLOCK:
        {
          uint8_t tray_id = received_message->get_can_signals()
                              .at(robast_can_msgs::can_signal::id::drawer_unlock::DRAWER_ID)
                              .get_data();
          tray_manager->unlock_lock(tray_id);
        }
        break;
        case robast_can_msgs::can_id::ELECTRICAL_DRAWER_TASK:
        {
          utils::EDrawerTask e_drawer_task =
            task_params->can_message_converter->convert_to_e_drawer_task(received_message.value());
          task_params->e_drawer->add_e_drawer_task_to_queue(e_drawer_task);
        }
        break;
        case robast_can_msgs::can_id::LED_HEADER:
        {
          led::LedHeader led_header =
            task_params->can_message_converter->convert_to_led_header(received_message.value());
          led_strip->initialize_led_state_change(led_header);
        }
        break;
        case robast_can_msgs::can_id::SINGLE_LED_STATE:
        {
          led::LedState led_state = task_params->can_message_converter->convert_to_led_state(received_message.value());
          led_strip->set_led_state(led_state);
        }
        break;
        case robast_can_msgs::can_id::TRAY_LED_BRIGHTNESS:
        {
          tray_manager->set_tray_led_brightness(
            received_message->get_can_signals()
              .at(robast_can_msgs::can_signal::id::tray_led_brightness::TRAY_ID)
              .get_data(),
            received_message->get_can_signals()
              .at(robast_can_msgs::can_signal::id::tray_led_brightness::TRAY_LED_ROW_INDEX)
              .get_data(),
            received_message->get_can_signals()
              .at(robast_can_msgs::can_signal::id::tray_led_brightness::TRAY_LED_STATE_BRIGHNESS)
              .get_data());
        }
        break;
        case robast_can_msgs::can_id::MODULE_CONFIG:
        {
          bool config_set_successfully =
            task_params->config_manager->set_config(received_message->get_can_signals()
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

    task_params->e_drawer->update_state();

    std::optional<robast_can_msgs::CanMessage> to_be_sent_message = task_params->e_drawer->can_out();

    if (to_be_sent_message.has_value())
    {
      task_params->can_controller->send_can_message(to_be_sent_message.value());
    }

    tray_manager->update_states();
  }
}

void setup()
{
  debug_setup(115200);
  debug_println("[Main]: Start...");

  // TODO: In the hardware design of v1 the pins of SDA and SCL are mixed up unfortunately for the port expander
  // TODO: In order to use the hardware anyway, we need to create two instances of the bus with different pin init
  std::shared_ptr<TwoWire> wire_onboard_led_driver = std::make_shared<TwoWire>(1);
  std::shared_ptr<TwoWire> wire_port_expander = std::make_shared<TwoWire>(2);
  wire_onboard_led_driver->begin(peripherals::i2c::I2C_SDA, peripherals::i2c::I2C_SCL);
  wire_port_expander->begin(peripherals::i2c::I2C_SDA_PORT_EXPANDER, peripherals::i2c::I2C_SCL_PORT_EXPANDER);

  std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper = std::make_shared<gpio::GpioWrapperPca9535>(
    wire_port_expander, slave_address_to_port_expander, pin_mapping_id_to_gpio_info, pin_mapping_id_to_port);

  std::shared_ptr<switch_lib::Switch> endstop_switch =
    std::make_shared<switch_lib::Switch>(gpio_wrapper,
                                         pin_id::SENSE_INPUT_DRAWER_1_CLOSED,
                                         SWITCH_PRESSED_THRESHOLD,
                                         switch_lib::Switch::normally_open,
                                         SWITCH_WEIGHT_NEW_VALUES);

  std::vector<TrayPinConfig> tray_pin_config = {
    {pin_id::LOCK_1_OPEN_CONTROL, pin_id::LOCK_1_CLOSE_CONTROL, pin_id::SENSE_INPUT_LID_1_CLOSED},
    {pin_id::LOCK_2_OPEN_CONTROL, pin_id::LOCK_2_CLOSE_CONTROL, pin_id::SENSE_INPUT_LID_2_CLOSED},
    {pin_id::LOCK_3_OPEN_CONTROL, pin_id::LOCK_3_CLOSE_CONTROL, pin_id::SENSE_INPUT_LID_3_CLOSED},
    {pin_id::LOCK_4_OPEN_CONTROL, pin_id::LOCK_4_CLOSE_CONTROL, pin_id::SENSE_INPUT_LID_4_CLOSED},
    {pin_id::LOCK_5_OPEN_CONTROL, pin_id::LOCK_5_CLOSE_CONTROL, pin_id::SENSE_INPUT_LID_5_CLOSED},
    {pin_id::LOCK_6_OPEN_CONTROL, pin_id::LOCK_6_CLOSE_CONTROL, pin_id::SENSE_INPUT_LID_6_CLOSED},
    {pin_id::LOCK_7_OPEN_CONTROL, pin_id::LOCK_7_CLOSE_CONTROL, pin_id::SENSE_INPUT_LID_7_CLOSED},
    {pin_id::LOCK_8_OPEN_CONTROL, pin_id::LOCK_8_CLOSE_CONTROL, pin_id::SENSE_INPUT_LID_8_CLOSED}};

  tray_manager = std::make_shared<TrayManager>(
    tray_pin_config, gpio_wrapper, wire_onboard_led_driver, SWITCH_PRESSED_THRESHOLD, SWITCH_WEIGHT_NEW_VALUES);

  auto set_led_driver_enable_pin_high = [gpio_wrapper]()
  {
    gpio_wrapper->set_pin_mode(pin_id::ENABLE_ONBOARD_LED_VDD, gpio_wrapper->get_gpio_output_pin_mode());
    gpio_wrapper->digital_write(pin_id::ENABLE_ONBOARD_LED_VDD, true);
  };
  tray_manager->init(set_led_driver_enable_pin_high);

  std::shared_ptr<robast_can_msgs::CanDb> can_db = std::make_shared<robast_can_msgs::CanDb>();
  std::shared_ptr<utils::CanMessageConverter> can_message_converter = std::make_shared<utils::CanMessageConverter>();

  led_strip = std::make_unique<led::LedStrip<peripherals::pinout::LED_PIXEL_PIN, NUM_OF_LEDS>>();

  std::shared_ptr<can_toolbox::CanController> can_controller = std::make_shared<can_toolbox::CanController>(
    MODULE_ID, can_db, peripherals::pinout::TWAI_TX_PIN, peripherals::pinout::TWAI_RX_PIN);

  can_queue_mutex = xSemaphoreCreateMutex();
  std::shared_ptr<utils::Queue<robast_can_msgs::CanMessage>> can_msg_queue =
    std::make_shared<utils::Queue<robast_can_msgs::CanMessage>>();

  std::shared_ptr<drawer::ElectricalDrawerConfig> drawer_config = std::make_shared<drawer::ElectricalDrawerConfig>();
  std::shared_ptr<motor::EncoderConfig> encoder_config = std::make_shared<motor::EncoderConfig>();
  std::shared_ptr<motor::MotorConfig> motor_config = std::make_shared<motor::MotorConfig>();
  std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config = std::make_shared<motor::MotorMonitorConfig>();

  std::shared_ptr<utils::ConfigManager> config_manager =
    std::make_shared<utils::ConfigManager>(drawer_config, encoder_config, motor_config, motor_monitor_config);

  stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
    .stepper_enn_tmc2209_pin_id = partial_drawer_controller::pin_id::STEPPER_1_ENN_TMC2209,
    .stepper_stdby_tmc2209_pin_id = partial_drawer_controller::pin_id::STEPPER_1_STDBY_TMC2209,
    .stepper_spread_pin_id = partial_drawer_controller::pin_id::STEPPER_1_SPREAD,
    .stepper_dir_pin_id = partial_drawer_controller::pin_id::STEPPER_1_DIR,
    .stepper_diag_pin_id = partial_drawer_controller::pin_id::STEPPER_1_DIAG,
    .stepper_index_pin_id = partial_drawer_controller::pin_id::STEPPER_1_INDEX,
    .stepper_step_pin_id = partial_drawer_controller::pin_id::STEPPER_1_STEP};

  std::shared_ptr<drawer::ElectricalDrawer> e_drawer =
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
                                               std::nullopt,
                                               drawer_config,
                                               encoder_config,
                                               motor_monitor_config);

  debug_println("[Main]: Finished setup()!");

  task_params_receive_can_msgs =
    std::make_shared<global_params::TaskParamsReceiveCanMsgs>(can_controller, can_msg_queue);

  xTaskCreatePinnedToCore(receive_can_msg_task_loop,          /* Task function. */
                          "Task1",                            /* name of task. */
                          10000,                              /* Stack size of task */
                          task_params_receive_can_msgs.get(), /* parameter of the task */
                          1,                                  /* priority of the task */
                          &Task1,                             /* Task handle to keep track of created task */
                          0);                                 /* pin task to core 0 */

  delay(100);

  task_params = std::make_shared<global_params::TaskParams>(can_db,
                                                            gpio_wrapper,
                                                            nullptr,
                                                            e_drawer,
                                                            drawer_config,
                                                            encoder_config,
                                                            motor_config,
                                                            motor_monitor_config,
                                                            config_manager,
                                                            endstop_switch,
                                                            can_message_converter,
                                                            can_controller,
                                                            can_msg_queue);

  xTaskCreatePinnedToCore(process_can_msgs_task_loop, /* Task function. */
                          "Task2",                    /* name of task. */
                          20000,                      /* Stack size of task */
                          task_params.get(),          /* parameter of the task */
                          1,                          /* priority of the task */
                          &Task2,                     /* Task handle to keep track of created task */
                          1);                         /* pin task to core 1 */
}

void loop()
{
  // In the setup we created two tasks that run in parallel, so we do not need to do anything here.
  // But as we are using the Arduino framework we need to have a loop function.
}
