#include <memory>

#include "can_toolbox/can_controller.hpp"
#include "drawer/electrical_drawer.hpp"
#include "drawer_controller/global.hpp"
#include "gpio/gpio_wrapper_pca9535.hpp"
#include "peripherals/gpio_defines.hpp"
#include "peripherals/pinout_defines.hpp"
#include "tray/partial_drawer_config_manager.hpp"
#include "tray/tray_manager.hpp"

// These are the very basic top level configurations for the drawer controller you need to set.
// Besides that there are a lot of other configs that are managed by the ConfigManager and can be set via CAN messages.
constexpr config::UserConfig USER_CONFIG{.module_version = config::version::CURA,
                                         .module_prefix = module_id::ModulePrefix::PARTIAL_DRAWER_10x40x8,
                                         .unique_module_id = 1,
                                         .lock_id = 0,
                                         .is_shaft_direction_inverted = true,
                                         .endstop_switch_type = switch_lib::Switch::normally_open,
                                         .use_color_fade = false,
                                         .allow_partial_led_changes = false};

constexpr config::ModuleHardwareConfig MODULE_HARDWARE_CONFIG =
  config::get_module_hardware_config<USER_CONFIG.module_version>(USER_CONFIG.module_prefix);

constexpr uint32_t MODULE_ID = module_id::generate_module_id(USER_CONFIG.module_prefix, USER_CONFIG.unique_module_id);

std::unique_ptr<led::LedStrip<peripherals::pinout::LED_PIXEL_PIN, MODULE_HARDWARE_CONFIG.total_num_of_leds>> led_strip;

std::shared_ptr<drawer::ElectricalDrawer> e_drawer;

std::shared_ptr<tray::TrayManager> tray_manager;

std::unique_ptr<can_toolbox::CanController> can_controller;

std::unique_ptr<tray::PartialDrawerConfigManager> config_manager;

stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
  .stepper_enn_tmc2209_pin_id = peripherals::pin_id::STEPPER_1_ENN_TMC2209,
  .stepper_stdby_tmc2209_pin_id = peripherals::pin_id::STEPPER_1_STDBY_TMC2209,
  .stepper_spread_pin_id = peripherals::pin_id::STEPPER_1_SPREAD,
  .stepper_dir_pin_id = peripherals::pin_id::STEPPER_1_DIR,
  .stepper_diag_pin_id = peripherals::pin_id::STEPPER_1_DIAG,
  .stepper_index_pin_id = peripherals::pin_id::STEPPER_1_INDEX,
  .stepper_step_pin_id = peripherals::pin_id::STEPPER_1_STEP};

/**********************************************************************************************************************
 * The program flow is organized as follows:
 * (1) setup() is called once at the beginning of the program
 * (2) The task loops contain a loop and run in parallel, whereby
 *    - receive_can_msg_task_loop() is responsible for receiving CAN messages and adding them to the queue
 *    - process_can_msgs_task_loop() is responsible for processing the CAN messages from the queue
 **********************************************************************************************************************/

void receive_can_msg_task_loop(void* pvParameters)
{
  for (;;)
  {
    std::optional<robast_can_msgs::CanMessage> received_message = can_controller->handle_receiving_can_msg();
    if (received_message.has_value())
    {
      if (xSemaphoreTake(can_queue_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
      {
        can_msg_queue->enqueue(received_message.value());
        xSemaphoreGive(can_queue_mutex);
      }
      else
      {
        serial_println_error("[Main]: Error: Could not take the mutex. This should not occur.");
      }
    }
  }
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
      serial_println_error("[Main]: Error: Could not take the mutex. This should not occur.");
    }

    if (received_message.has_value())
    {
      switch (received_message->get_id())
      {
        case robast_can_msgs::can_id::DRAWER_UNLOCK:
        {
          const uint8_t tray_id = received_message->get_can_signals()
                                    .at(robast_can_msgs::can_signal::id::drawer_unlock::DRAWER_ID)
                                    .get_data();
          tray_manager->unlock_lock(tray_id);
        }
        break;
        case robast_can_msgs::can_id::ELECTRICAL_DRAWER_TASK:
        {
          const utils::EDrawerTask e_drawer_task =
            can_message_converter->convert_to_e_drawer_task(received_message.value());
          e_drawer->add_e_drawer_task_to_queue(e_drawer_task);
        }
        break;
        case robast_can_msgs::can_id::LED_HEADER:
        {
          const led::LedHeader led_header = can_message_converter->convert_to_led_header(received_message.value());
          led_strip->initialize_led_state_change(led_header);
        }
        break;
        case robast_can_msgs::can_id::LED_STATE:
        {
          const led::LedState led_state = can_message_converter->convert_to_led_state(received_message.value());
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
          const bool config_set_successfully =
            config_manager->set_config(received_message->get_can_signals()
                                         .at(robast_can_msgs::can_signal::id::module_config::CONFIG_ID)
                                         .get_data(),
                                       received_message->get_can_signals()
                                         .at(robast_can_msgs::can_signal::id::module_config::CONFIG_VALUE)
                                         .get_data());
          if (!config_set_successfully)
          {
            serial_println_warning("[Main]: Warning - Tried to set config for invalid config id!");
          }
          else
          {
            serial_printf_green("[Main]: Successfully set config with id %d to value %d\n",
                                received_message->get_can_signals()
                                  .at(robast_can_msgs::can_signal::id::module_config::CONFIG_ID)
                                  .get_data(),
                                received_message->get_can_signals()
                                  .at(robast_can_msgs::can_signal::id::module_config::CONFIG_VALUE)
                                  .get_data());
          }
        }
        case robast_can_msgs::can_id::ELECTRICAL_DRAWER_MOTOR_CONTROL:
        {
          const bool enable_motor =
            received_message->get_can_signals()
              .at(robast_can_msgs::can_signal::id::electrical_drawer_motor_control::ENABLE_MOTOR)
              .get_data() == 1;
          const uint8_t motor_id = received_message->get_can_signals()
                                     .at(robast_can_msgs::can_signal::id::electrical_drawer_motor_control::MOTOR_ID)
                                     .get_data();
          motion_controller->set_motor_driver_state(enable_motor, motor_id);
          can_utils->enqueue_e_drawer_motor_control_msg(
            MODULE_ID, motor_id, enable_motor, drawer::CONFIRM_MOTOR_CONTROL_CHANGE);
        }
        default:
          serial_println_warning("[Main]: Received unsupported CAN message.");
          break;
      }
    }

    led_strip->handle_led_control();

    e_drawer->update_state();

    heartbeat->generate_heartbeat();

    std::optional<robast_can_msgs::CanMessage> to_be_sent_message = can_utils->get_element_from_feedback_msg_queue();

    if (to_be_sent_message.has_value())
    {
      can_controller->send_can_message(to_be_sent_message.value());
    }

    tray_manager->update_states();
  }
}

void setup()
{
  Serial.begin(115200);
  debug_printf_green("[Main]: Start the module with the module id: %d\n", MODULE_ID);

  // TODO: In the hardware design of v1 the pins of SDA and SCL are mixed up unfortunately for the port expander
  // TODO: In order to use the hardware anyway, we need to create two instances of the bus with different pin init
  std::shared_ptr<TwoWire> wire_onboard_led_driver = std::make_shared<TwoWire>(1);
  std::shared_ptr<TwoWire> wire_port_expander = std::make_shared<TwoWire>(2);
  wire_onboard_led_driver->begin(peripherals::i2c::I2C_SDA, peripherals::i2c::I2C_SCL);
  wire_port_expander->begin(peripherals::i2c::I2C_SDA_PORT_EXPANDER, peripherals::i2c::I2C_SCL_PORT_EXPANDER);

  gpio_wrapper = std::make_shared<gpio::GpioWrapperPca9535>(wire_port_expander,
                                                            peripherals::slave_address_to_port_expander,
                                                            peripherals::pin_mapping_id_to_gpio_info,
                                                            peripherals::pin_mapping_id_to_slave_address_by_port);

  endstop_switch = std::make_shared<switch_lib::Switch>(gpio_wrapper,
                                                        peripherals::pin_id::SENSE_INPUT_DRAWER_1_CLOSED,
                                                        SWITCH_PRESSED_THRESHOLD,
                                                        USER_CONFIG.endstop_switch_type,
                                                        SWITCH_WEIGHT_NEW_VALUES);

  can_db = std::make_shared<robast_can_msgs::CanDb>();
  can_message_converter = std::make_unique<utils::CanMessageConverter>();
  can_utils = std::make_shared<can_toolbox::CanUtils>(can_db);

  led_strip =
    std::make_unique<led::LedStrip<peripherals::pinout::LED_PIXEL_PIN, MODULE_HARDWARE_CONFIG.total_num_of_leds>>(
      MODULE_ID, USER_CONFIG.use_color_fade, USER_CONFIG.allow_partial_led_changes, can_utils);

  can_queue_mutex = xSemaphoreCreateMutex();
  can_msg_queue = std::make_unique<utils::Queue<robast_can_msgs::CanMessage>>();

  can_controller = std::make_unique<can_toolbox::CanController>(
    MODULE_ID, can_db, peripherals::pinout::TWAI_TX_PIN, peripherals::pinout::TWAI_RX_PIN);

  config_manager = std::make_unique<tray::PartialDrawerConfigManager>();
  config_manager->set_config(module_config::motor::IS_SHAFT_DIRECTION_INVERTED,
                             USER_CONFIG.is_shaft_direction_inverted ? 1 : 0);
  config_manager->print_all_configs();

  rotating_file_logger =
    std::make_shared<logging::RotatingFileHandler>(config_manager->get_rotating_file_handler_config());
  rotating_file_logger->print_all_logs();

  heartbeat = std::make_shared<watchdog::Heartbeat>(MODULE_ID, can_utils, config_manager->get_heartbeat_config());

  // The partial drawer currently has no drawer lock
  const std::optional<std::shared_ptr<lock::ElectricalDrawerLock>> drawer_lock = std::nullopt;

  motion_controller = std::make_shared<drawer::MotionController>(
    MODULE_ID,
    USER_CONFIG.lock_id,
    MODULE_HARDWARE_CONFIG.use_encoder,
    gpio_wrapper->get_gpio_num_for_pin_id(peripherals::pin_id::STEPPER_1_ENCODER_A),
    gpio_wrapper->get_gpio_num_for_pin_id(peripherals::pin_id::STEPPER_1_ENCODER_B),
    STEPPER_MOTOR_1_ADDRESS,
    config_manager->get_encoder_config(),
    config_manager->get_motor_config(),
    config_manager->get_motor_monitor_config(),
    stepper_1_pin_id_config,
    gpio_wrapper,
    config_manager->get_drawer_config());

  e_drawer = std::make_shared<drawer::ElectricalDrawer>(MODULE_ID,
                                                        USER_CONFIG.lock_id,
                                                        can_utils,
                                                        endstop_switch,
                                                        config_manager->get_drawer_config(),
                                                        motion_controller,
                                                        drawer_lock);

  std::vector<tray::TrayPinConfig> tray_pin_config = {{peripherals::pin_id::LOCK_1_OPEN_CONTROL,
                                                       peripherals::pin_id::LOCK_1_CLOSE_CONTROL,
                                                       peripherals::pin_id::SENSE_INPUT_LID_1_CLOSED},
                                                      {peripherals::pin_id::LOCK_2_OPEN_CONTROL,
                                                       peripherals::pin_id::LOCK_2_CLOSE_CONTROL,
                                                       peripherals::pin_id::SENSE_INPUT_LID_2_CLOSED},
                                                      {peripherals::pin_id::LOCK_3_OPEN_CONTROL,
                                                       peripherals::pin_id::LOCK_3_CLOSE_CONTROL,
                                                       peripherals::pin_id::SENSE_INPUT_LID_3_CLOSED},
                                                      {peripherals::pin_id::LOCK_4_OPEN_CONTROL,
                                                       peripherals::pin_id::LOCK_4_CLOSE_CONTROL,
                                                       peripherals::pin_id::SENSE_INPUT_LID_4_CLOSED},
                                                      {peripherals::pin_id::LOCK_5_OPEN_CONTROL,
                                                       peripherals::pin_id::LOCK_5_CLOSE_CONTROL,
                                                       peripherals::pin_id::SENSE_INPUT_LID_5_CLOSED},
                                                      {peripherals::pin_id::LOCK_6_OPEN_CONTROL,
                                                       peripherals::pin_id::LOCK_6_CLOSE_CONTROL,
                                                       peripherals::pin_id::SENSE_INPUT_LID_6_CLOSED},
                                                      {peripherals::pin_id::LOCK_7_OPEN_CONTROL,
                                                       peripherals::pin_id::LOCK_7_CLOSE_CONTROL,
                                                       peripherals::pin_id::SENSE_INPUT_LID_7_CLOSED},
                                                      {peripherals::pin_id::LOCK_8_OPEN_CONTROL,
                                                       peripherals::pin_id::LOCK_8_CLOSE_CONTROL,
                                                       peripherals::pin_id::SENSE_INPUT_LID_8_CLOSED}};

  tray_manager = std::make_shared<tray::TrayManager>(tray_pin_config,
                                                     gpio_wrapper,
                                                     wire_onboard_led_driver,
                                                     motion_controller,
                                                     config_manager->get_motor_monitor_config(),
                                                     config_manager->get_tray_manager_config(),
                                                     SWITCH_PRESSED_THRESHOLD,
                                                     SWITCH_WEIGHT_NEW_VALUES);

  auto set_led_driver_enable_pin_high = []()
  {
    gpio_wrapper->set_pin_mode(peripherals::pin_id::ENABLE_ONBOARD_LED_VDD, interfaces::gpio::IS_OUTPUT);
    gpio_wrapper->digital_write(peripherals::pin_id::ENABLE_ONBOARD_LED_VDD, true);
  };
  tray_manager->init(set_led_driver_enable_pin_high);

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

  debug_printf_green("[Main]: Finished setup()!\n");
}

void loop()
{
  // In the setup we created two tasks that run in parallel, so we do not need to do anything here.
  // But as we are using the Arduino framework we need to have a loop function.
}
