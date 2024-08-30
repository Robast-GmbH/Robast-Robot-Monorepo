#include <memory>

#include "drawer_controller/global.hpp"
#include "led/led_strip.hpp"

// VERY IMPORTANT: Set the hardware version of the drawer controller pcb here (currently 3 and 4 are supported):
#define HARDWARE_VERSION 4
#if HARDWARE_VERSION == 3
#include "can_toolbox/can_controller_mcp2515.hpp"
#include "gpio/gpio_wrapper_pca9554.hpp"
#include "peripherals/gpio_defines_v3.hpp"
#include "peripherals/pinout_defines_v3.hpp"
#elif HARDWARE_VERSION == 4
#include "can_toolbox/can_controller.hpp"
#include "gpio/gpio_wrapper_pca9535.hpp"
#include "peripherals/gpio_defines_v4.hpp"
#include "peripherals/pinout_defines_v4.hpp"
#else
#error "Unsupported HARDWARE_VERSION. Please define HARDWARE_VERSION as 3 or 4."
#endif

// These are the very basic top level configurations for the drawer controller you need to set.
constexpr bool MODULE_CONTAINS_A_DRAWER = false;
constexpr bool IS_ELECTRICAL_DRAWER = false;
constexpr uint32_t MODULE_ID = 0;
constexpr uint8_t LOCK_ID = 0;
constexpr bool USE_ENCODER = false;
constexpr bool IS_SHAFT_DIRECTION_INVERTED = false;
constexpr switch_lib::Switch::SwitchType ENDSTOP_SWITCH_TYPE = switch_lib::Switch::normally_open;
// LED CONFIGS
constexpr uint8_t NUM_OF_LEDS = 128;   // 18 LEDs at the old drawer and 21 LEDs at the new drawer
constexpr bool USE_COLOR_FADE = false;

std::unique_ptr<led::LedStrip<peripherals::pinout::LED_PIXEL_PIN, NUM_OF_LEDS>> led_strip;

// Initialize can_controller based on HARDWARE_VERSION
#if HARDWARE_VERSION == 3
std::unique_ptr<can_toolbox::CanController<peripherals::pinout::SPI_MISO,
                                           peripherals::pinout::SPI_MOSI,
                                           peripherals::pinout::SPI_CLK,
                                           peripherals::pinout::SPI_CS,
                                           peripherals::pinout::MCP2515_INT>>
  can_controller;
#elif HARDWARE_VERSION == 4
std::unique_ptr<can_toolbox::CanController> can_controller;
#endif

stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
  .stepper_enn_tmc2209_pin_id = gpio_defines::pin_id::STEPPER_1_ENN_TMC2209,
  .stepper_stdby_tmc2209_pin_id = gpio_defines::pin_id::STEPPER_1_STDBY_TMC2209,
  .stepper_spread_pin_id = gpio_defines::pin_id::STEPPER_1_SPREAD,
  .stepper_dir_pin_id = gpio_defines::pin_id::STEPPER_1_DIR,
  .stepper_diag_pin_id = gpio_defines::pin_id::STEPPER_1_DIAG,
  .stepper_index_pin_id = gpio_defines::pin_id::STEPPER_1_INDEX,
  .stepper_step_pin_id = gpio_defines::pin_id::STEPPER_1_STEP};

/**********************************************************************************************************************
 * The program flow is organized as follows:
 * (1) setup() is called once at the beginning of the program
 * (2) The task loops contain a loop and run in parallel, whereby
 *    - receive_can_msg_task_loop() is responsible for receiving CAN messages and adding them to the queue
 *    - process_can_msgs_task_loop() is responsible for processing the CAN messages from the queue
 **********************************************************************************************************************/

void add_can_msg_to_queue(std::optional<robast_can_msgs::CanMessage>& received_message)
{
  if (xSemaphoreTake(can_queue_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
  {
    debug_println("[Main]: Received CAN message and adding it to the queue.");
    can_msg_queue->enqueue(received_message.value());
    xSemaphoreGive(can_queue_mutex);
  }
  else
  {
    Serial.println("[Main]: Error: Could not take the mutex. This should not occur.");
  }
}

void receive_can_msg_task_loop_hw_v3(void)
{
  uint8_t minimal_loop_time_in_ms = MINIMAL_LOOP_TIME_IN_MS;
  uint8_t num_of_led_changes = 0;

  for (;;)
  {
    TickType_t current_tick_count = xTaskGetTickCount();

    std::optional<robast_can_msgs::CanMessage> received_message = can_controller->handle_receiving_can_msg();
    if (received_message.has_value())
    {
      add_can_msg_to_queue(received_message);

      if (received_message.value().get_id() == robast_can_msgs::can_id::LED_HEADER)
      {
        // If a new LED header is received, we set the minimal loop time to 0 to handle the LED changes faster
        minimal_loop_time_in_ms = 0;
        num_of_led_changes = received_message.value()
                               .get_can_signals()
                               .at(robast_can_msgs::can_signal::id::led_header::NUM_OF_LEDS)
                               .get_data();
      }
    }

    unsigned long loop_time = pdTICKS_TO_MS(xTaskGetTickCount() - current_tick_count);

    if (loop_time < minimal_loop_time_in_ms)
    {
      // Task yielding to give IDLE task a chance to run and reset watchdog timer
      vTaskDelay(pdMS_TO_TICKS(minimal_loop_time_in_ms - pdTICKS_TO_MS(loop_time)));
    }
    if (num_of_led_changes > 0)
    {
      num_of_led_changes--;
    }
    if (num_of_led_changes == 0)
    {
      minimal_loop_time_in_ms = MINIMAL_LOOP_TIME_IN_MS;
    }
  }
}

void receive_can_msg_task_loop_hw_v4(void)
{
  for (;;)
  {
    std::optional<robast_can_msgs::CanMessage> received_message = can_controller->handle_receiving_can_msg();
    if (received_message.has_value())
    {
      add_can_msg_to_queue(received_message);
    }
  }
}

void receive_can_msg_task_loop(void* pvParameters)
{
#if HARDWARE_VERSION == 3
  // Hardware Version 3 needs a different task loop with extra task yielding because it uses SPI CAN Controller
  receive_can_msg_task_loop_hw_v3();
#elif HARDWARE_VERSION == 4
  // Hardware Version 4 uses TWAI CAN Controller and does not need extra task yielding because it uses xQueueReceive
  // which has a task yielding wait
  receive_can_msg_task_loop_hw_v4();
#endif
}

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
          i_drawer->unlock();
        }
        break;
        case robast_can_msgs::can_id::ELECTRICAL_DRAWER_TASK:
        {
          utils::EDrawerTask e_drawer_task = can_message_converter->convert_to_e_drawer_task(received_message.value());
          i_drawer->add_e_drawer_task_to_queue(e_drawer_task);
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

    if (MODULE_CONTAINS_A_DRAWER)
    {
      i_drawer->update_state();
    }

    std::optional<robast_can_msgs::CanMessage> to_be_sent_message = i_drawer->can_out();

    if (to_be_sent_message.has_value())
    {
      can_controller->send_can_message(to_be_sent_message.value());
    }
  }
}

void setup()
{
  Serial.begin(115200);
  debug_println("[Main]: Start...");

  std::shared_ptr<TwoWire> wire_port_expander = std::make_shared<TwoWire>(1);
  wire_port_expander->begin(peripherals::i2c::I2C_SDA, peripherals::i2c::I2C_SCL);

#if HARDWARE_VERSION == 3
  gpio_wrapper = std::make_shared<gpio::GpioWrapperPca9554>(wire_port_expander,
                                                            gpio_defines::slave_address_to_port_expander,
                                                            gpio_defines::pin_mapping_id_to_gpio_info,
                                                            gpio_defines::pin_mapping_id_to_slave_address_by_register);
#elif HARDWARE_VERSION == 4
  gpio_wrapper = std::make_shared<gpio::GpioWrapperPca9535>(wire_port_expander,
                                                            gpio_defines::slave_address_to_port_expander,
                                                            gpio_defines::pin_mapping_id_to_gpio_info,
                                                            gpio_defines::pin_mapping_id_to_slave_address_by_port);
#endif

  endstop_switch = std::make_shared<switch_lib::Switch>(gpio_wrapper,
                                                        gpio_defines::pin_id::SENSE_INPUT_DRAWER_1_CLOSED,
                                                        SWITCH_PRESSED_THRESHOLD,
                                                        ENDSTOP_SWITCH_TYPE,
                                                        SWITCH_WEIGHT_NEW_VALUES);

  can_db = std::make_shared<robast_can_msgs::CanDb>();
  can_message_converter = std::make_unique<utils::CanMessageConverter>();

  led_strip = std::make_unique<led::LedStrip<peripherals::pinout::LED_PIXEL_PIN, NUM_OF_LEDS>>(USE_COLOR_FADE);

  // Very important to initialize this before the can_controller is created
  can_queue_mutex = xSemaphoreCreateMutex();
  can_msg_queue = std::make_unique<utils::Queue<robast_can_msgs::CanMessage>>();

#if HARDWARE_VERSION == 3
  can_controller = std::make_unique<can_toolbox::CanController<peripherals::pinout::SPI_MISO,
                                                               peripherals::pinout::SPI_MOSI,
                                                               peripherals::pinout::SPI_CLK,
                                                               peripherals::pinout::SPI_CS,
                                                               peripherals::pinout::MCP2515_INT>>(
    MODULE_ID, can_db, gpio_wrapper, gpio_defines::pin_id::OE_TXB0104);
#elif HARDWARE_VERSION == 4
  can_controller = std::make_unique<can_toolbox::CanController>(
    MODULE_ID, can_db, peripherals::pinout::TWAI_TX_PIN, peripherals::pinout::TWAI_RX_PIN);
#endif

  drawer_lock = std::make_shared<lock::ElectricalDrawerLock>(gpio_wrapper,
                                                             gpio_defines::pin_id::LOCK_1_OPEN_CONTROL,
                                                             gpio_defines::pin_id::LOCK_1_CLOSE_CONTROL,
                                                             gpio_defines::pin_id::LOCK_1_SENSE,
                                                             SWITCH_PRESSED_THRESHOLD,
                                                             SWITCH_WEIGHT_NEW_VALUES);

  drawer_config = std::make_shared<drawer::ElectricalDrawerConfig>();
  encoder_config = std::make_shared<motor::EncoderConfig>();
  motor_config = std::make_shared<motor::MotorConfig>();
  motor_monitor_config = std::make_shared<motor::MotorMonitorConfig>();

  config_manager =
    std::make_unique<utils::ConfigManager>(drawer_config, encoder_config, motor_config, motor_monitor_config);
  config_manager->set_config(module_config::motor::IS_SHAFT_DIRECTION_INVERTED, IS_SHAFT_DIRECTION_INVERTED ? 1 : 0);

  if (IS_ELECTRICAL_DRAWER)
  {
    i_drawer = std::make_shared<drawer::ElectricalDrawer>(
      MODULE_ID,
      LOCK_ID,
      can_db,
      gpio_wrapper,
      stepper_1_pin_id_config,
      USE_ENCODER,
      gpio_wrapper->get_gpio_num_for_pin_id(gpio_defines::pin_id::STEPPER_1_ENCODER_A),
      gpio_wrapper->get_gpio_num_for_pin_id(gpio_defines::pin_id::STEPPER_1_ENCODER_B),
      STEPPER_MOTOR_1_ADDRESS,
      motor_config,
      endstop_switch,
      drawer_lock,
      drawer_config,
      encoder_config,
      motor_monitor_config);
  }
  else
  {
    i_drawer = std::make_shared<drawer::ManualDrawer>(MODULE_ID, LOCK_ID, can_db, endstop_switch, drawer_lock);
  }

  // Initialize CAN Controller right before can task receive loop is started, otherwise rx_queue might overflow
  can_controller->initialize_can_controller();

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

  debug_println("[Main]: Finished setup()!");
}

void loop()
{
  // In the setup we created two tasks that run in parallel, so we do not need to do anything here.
  // But as we are using the Arduino framework we need to have a loop function.
}
