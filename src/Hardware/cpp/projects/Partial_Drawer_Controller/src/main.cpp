#include <memory>

#include "drawer_controller/global.hpp"
#include "lock/tray_manager.hpp"

#define MODULE_ID   6
#define LOCK_ID     0
#define USE_ENCODER true

#define NUM_OF_LEDS 21

std::unique_ptr<drawer_controller::LedStrip<LED_PIXEL_PIN, NUM_OF_LEDS>> led_strip;

std::shared_ptr<partial_drawer_controller::TrayManager> tray_manager;

/**********************************************************************************************************************
 * The program flow is organized as follows:
 * (1) setup() is called once at the beginning of the program
 * (2) The task loops contain a loop and run in parallel, whereby
 *    - receive_can_msg_task_loop() is responsible for receiving CAN messages and adding them to the queue
 *    - process_can_msgs_task_loop() is responsible for processing the CAN messages from the queue
 **********************************************************************************************************************/

void process_can_msgs_task_loop(void* pvParameters)
{
  for (;;)
  {
    std::optional<robast_can_msgs::CanMessage> received_message;
    if (xSemaphoreTake(can_queue_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
    {
      received_message = can_msg_queue->get_element_from_queue();
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
          drawer_controller::EDrawerTask e_drawer_task = data_mapper->create_e_drawer_task(received_message.value());
          drawer->add_e_drawer_task_to_queue(e_drawer_task);
        }
        break;
        case robast_can_msgs::can_id::LED_HEADER:
        {
          drawer_controller::LedHeader led_header = data_mapper->create_led_header(received_message.value());
          led_strip->initialize_led_state_change(led_header);
        }
        break;
        case robast_can_msgs::can_id::SINGLE_LED_STATE:
        {
          drawer_controller::LedState led_state = data_mapper->create_led_state(received_message.value());
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
          config_manager->set_config(received_message->get_can_signals()
                                       .at(robast_can_msgs::can_signal::id::module_config::CONFIG_ID)
                                       .get_data(),
                                     received_message->get_can_signals()
                                       .at(robast_can_msgs::can_signal::id::module_config::CONFIG_VALUE)
                                       .get_data());
        }
        default:
          debug_println("[Main]: Received unsupported CAN message.");
          break;
      }
    }

    led_strip->handle_led_control();

    drawer->update_state();

    std::optional<robast_can_msgs::CanMessage> to_be_sent_message = drawer->can_out();

    if (to_be_sent_message.has_value())
    {
      can_controller->send_can_message(to_be_sent_message.value());
    }

    tray_manager->update_states();

    // end timer to track time for one loop iteration
    // unsigned long end_time = millis();
    // unsigned long duration = end_time - start_time;
    // debug_printf("[Main]: Duration of one loop iteration: %lu ms. Free heap: %u\n", duration, ESP.getFreeHeap());
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
  wire_onboard_led_driver->begin(I2C_SDA, I2C_SCL);
  wire_port_expander->begin(I2C_SDA_PORT_EXPANDER, I2C_SCL_PORT_EXPANDER);

  gpio_wrapper =
    std::make_shared<drawer_controller::GpioWrapperPca9535>(wire_port_expander,
                                                            partial_drawer_controller::port_expanders,
                                                            partial_drawer_controller::pin_mapping_id_to_gpio_info,
                                                            partial_drawer_controller::pin_mapping_id_to_port);

  endstop_switch = std::make_shared<drawer_controller::Switch>(gpio_wrapper,
                                                               SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID,
                                                               SWITCH_PRESSED_THRESHOLD,
                                                               drawer_controller::Switch::normally_open,
                                                               SWITCH_WEIGHT_NEW_VALUES);

  std::vector<partial_drawer_controller::TrayPinConfig> tray_pin_config = {
    {LOCK_1_OPEN_CONTROL_PIN_ID, LOCK_1_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_1_CLOSED_PIN_ID},
    {LOCK_2_OPEN_CONTROL_PIN_ID, LOCK_2_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_2_CLOSED_PIN_ID},
    {LOCK_3_OPEN_CONTROL_PIN_ID, LOCK_3_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_3_CLOSED_PIN_ID},
    {LOCK_4_OPEN_CONTROL_PIN_ID, LOCK_4_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_4_CLOSED_PIN_ID},
    {LOCK_5_OPEN_CONTROL_PIN_ID, LOCK_5_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_5_CLOSED_PIN_ID},
    {LOCK_6_OPEN_CONTROL_PIN_ID, LOCK_6_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_6_CLOSED_PIN_ID},
    {LOCK_7_OPEN_CONTROL_PIN_ID, LOCK_7_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_7_CLOSED_PIN_ID},
    {LOCK_8_OPEN_CONTROL_PIN_ID, LOCK_8_CLOSE_CONTROL_PIN_ID, SENSE_INPUT_LID_8_CLOSED_PIN_ID}};

  tray_manager = std::make_shared<partial_drawer_controller::TrayManager>(
    tray_pin_config, gpio_wrapper, wire_onboard_led_driver, SWITCH_PRESSED_THRESHOLD, SWITCH_WEIGHT_NEW_VALUES);

  auto set_led_driver_enable_pin_high = []()
  {
    gpio_wrapper->set_pin_mode(ENABLE_ONBOARD_LED_VDD_PIN_ID, gpio_wrapper->get_gpio_output_pin_mode());
    gpio_wrapper->digital_write(ENABLE_ONBOARD_LED_VDD_PIN_ID, true);
  };
  tray_manager->init(set_led_driver_enable_pin_high);

  can_db = std::make_shared<robast_can_msgs::CanDb>();
  data_mapper = std::make_unique<drawer_controller::DataMapper>();

  led_strip = std::make_unique<drawer_controller::LedStrip<LED_PIXEL_PIN, NUM_OF_LEDS>>();

  can_controller = std::make_unique<drawer_controller::CanController>(MODULE_ID, can_db, TWAI_TX_PIN, TWAI_RX_PIN);

  can_queue_mutex = xSemaphoreCreateMutex();
  can_msg_queue = std::make_unique<drawer_controller::Queue<robast_can_msgs::CanMessage>>();

  drawer_config = std::make_shared<drawer_controller::ElectricalDrawerConfig>();
  encoder_config = std::make_shared<drawer_controller::EncoderConfig>();
  motor_config = std::make_shared<drawer_controller::MotorConfig>();
  motor_monitor_config = std::make_shared<drawer_controller::MotorMonitorConfig>();

  config_manager = std::make_unique<drawer_controller::ConfigManager>(
    drawer_config, encoder_config, motor_config, motor_monitor_config);

  drawer = std::make_shared<drawer_controller::ElectricalDrawer>(
    MODULE_ID,
    LOCK_ID,
    can_db,
    gpio_wrapper,
    stepper_1_pin_id_config,
    USE_ENCODER,
    gpio_wrapper->get_gpio_num_for_pin_id(STEPPER_1_ENCODER_A_PIN_ID),
    gpio_wrapper->get_gpio_num_for_pin_id(STEPPER_1_ENCODER_B_PIN_ID),
    STEPPER_MOTOR_1_ADDRESS,
    motor_config,
    endstop_switch,
    std::nullopt,
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
