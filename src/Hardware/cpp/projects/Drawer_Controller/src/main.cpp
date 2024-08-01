#include <memory>

#include "drawer_controller/global.hpp"
#include "led/led_strip.hpp"

#define MODULE_ID   6
#define LOCK_ID     0
#define USE_ENCODER 0

#define NUM_OF_LEDS 18

std::unique_ptr<drawer_controller::LedStrip<LED_PIXEL_PIN, NUM_OF_LEDS>> led_strip;

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
      Serial.println("Error: Could not take the mutex. This should not occur.");
    }

    if (received_message.has_value())
    {
      switch (received_message->get_id())
      {
        case robast_can_msgs::can_id::DRAWER_UNLOCK:
        {
          drawer->unlock();
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
  }
}

void setup()
{
  debug_setup(115200);
  debug_println("[Main]: Start...");

  std::shared_ptr<TwoWire> wire_port_expander = std::make_shared<TwoWire>(2);
  wire_port_expander->begin(I2C_SDA, I2C_SCL);

  gpio_wrapper = std::make_shared<drawer_controller::GpioWrapperPca9535>(wire_port_expander,
                                                                         drawer_controller::port_expanders,
                                                                         drawer_controller::pin_mapping_id_to_gpio_info,
                                                                         drawer_controller::pin_mapping_id_to_port);

  endstop_switch = std::make_shared<drawer_controller::Switch>(gpio_wrapper,
                                                               SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID,
                                                               SWITCH_PRESSED_THRESHOLD,
                                                               drawer_controller::Switch::normally_closed,
                                                               SWITCH_WEIGHT_NEW_VALUES);

  can_db = std::make_shared<robast_can_msgs::CanDb>();
  data_mapper = std::make_unique<drawer_controller::DataMapper>();

  led_strip = std::make_unique<drawer_controller::LedStrip<LED_PIXEL_PIN, NUM_OF_LEDS>>();

  can_controller = std::make_unique<drawer_controller::CanController>(MODULE_ID, can_db, TWAI_TX_PIN, TWAI_RX_PIN);
  can_controller->initialize_can_controller();

  can_queue_mutex = xSemaphoreCreateMutex();
  can_msg_queue = std::make_unique<drawer_controller::Queue<robast_can_msgs::CanMessage>>();

  drawer_lock = std::make_shared<drawer_controller::ElectricalDrawerLock>(gpio_wrapper,
                                                                          LOCK_1_OPEN_CONTROL_PIN_ID,
                                                                          LOCK_1_CLOSE_CONTROL_PIN_ID,
                                                                          LOCK_1_SENSE_PIN_ID,
                                                                          SWITCH_PRESSED_THRESHOLD,
                                                                          SWITCH_WEIGHT_NEW_VALUES);

  drawer_configs = std::make_shared<drawer_controller::ElectricalDrawerConfigs>();
  encoder_configs = std::make_shared<drawer_controller::EncoderConfigs>();
  motor_configs = std::make_shared<drawer_controller::MotorConfigs>();
  motor_monitor_configs = std::make_shared<drawer_controller::MotorMonitorConfigs>();

  config_manager = std::make_unique<drawer_controller::ConfigManager>(
    drawer_configs, encoder_configs, motor_configs, motor_monitor_configs);

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
    motor_configs,
    endstop_switch,
    drawer_lock,
    drawer_configs,
    encoder_configs,
    motor_monitor_configs);
  drawer->init();

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
