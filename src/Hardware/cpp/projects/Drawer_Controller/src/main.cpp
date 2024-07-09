#include <memory>

#include "can_toolbox/can_controller.hpp"
#include "debug/debug.hpp"
#include "drawer/electrical_drawer.hpp"
#include "gpio/gpio_wrapper_pca9535.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "led/led_strip.hpp"
#include "peripherals/gpio_defines.hpp"
#include "switch/switch.hpp"
#include "utils/data_mapper.hpp"
#include "utils/queue.hpp"

#define MODULE_ID               6
#define LOCK_ID                 0
#define STEPPER_MOTOR_1_ADDRESS 0
#define USE_ENCODER             0

#define NUM_OF_LEDS 18

#define SWITCH_PRESSED_THRESHOLD 0.9

TaskHandle_t Task1;
TaskHandle_t Task2;

SemaphoreHandle_t can_queue_mutex = NULL;   // Create a mutex object

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

std::shared_ptr<drawer_controller::Switch> endstop_switch;

std::unique_ptr<drawer_controller::LedStrip<LED_PIXEL_PIN, NUM_OF_LEDS>> led_strip;

std::unique_ptr<drawer_controller::DataMapper> data_mapper;

std::unique_ptr<drawer_controller::CanController> can_controller;

// shared resource, so we need a mutex for this
std::unique_ptr<drawer_controller::Queue<robast_can_msgs::CanMessage>> can_msg_queue;

void task_1_loop(void* pvParameters)
{
  for (;;)
  {
    std::optional<robast_can_msgs::CanMessage> received_message = can_controller->handle_receiving_can_msg();
    if (received_message.has_value())
    {
      if (xSemaphoreTake(can_queue_mutex, portMAX_DELAY) == pdTRUE)
      {
        can_msg_queue->add_element_to_queue(received_message.value());
        xSemaphoreGive(can_queue_mutex);
      }
      else
      {
        Serial.println("Error: Could not take the mutex. This should not occur.");
      }
    }
  }
}

void task_2_loop(void* pvParameters)
{
  for (;;)
  {
    std::optional<robast_can_msgs::CanMessage> received_message;
    if (xSemaphoreTake(can_queue_mutex, portMAX_DELAY) == pdTRUE)
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
        case CAN_ID_DRAWER_UNLOCK:
        {
          uint8_t tray_id = received_message->get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data();
          // TODO: Implement unlock
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
}

void setup()
{
  debug_setup(115200);
  debug_println("\nStart...");

  std::shared_ptr<TwoWire> wire_port_expander = std::make_shared<TwoWire>(2);
  wire_port_expander->begin(I2C_SDA, I2C_SCL);

  gpio_wrapper = std::make_shared<drawer_controller::GpioWrapperPca9535>(wire_port_expander,
                                                                         drawer_controller::port_expanders,
                                                                         drawer_controller::pin_mapping_id_to_gpio_info,
                                                                         drawer_controller::pin_mapping_id_to_port);

  endstop_switch = std::make_shared<drawer_controller::Switch>(gpio_wrapper,
                                                               SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID,
                                                               SWITCH_PRESSED_THRESHOLD,
                                                               drawer_controller::Switch::normally_closed);

  can_db = std::make_shared<robast_can_msgs::CanDb>();
  data_mapper = std::make_unique<drawer_controller::DataMapper>();

  led_strip = std::make_unique<drawer_controller::LedStrip<LED_PIXEL_PIN, NUM_OF_LEDS>>();

  can_controller = std::make_unique<drawer_controller::CanController>(MODULE_ID, can_db, TWAI_TX_PIN, TWAI_RX_PIN);
  can_controller->initialize_can_controller();

  can_queue_mutex = xSemaphoreCreateMutex();
  can_msg_queue = std::make_unique<drawer_controller::Queue<robast_can_msgs::CanMessage>>();

  e_drawer_0 = std::make_shared<drawer_controller::ElectricalDrawer>(
    MODULE_ID,
    LOCK_ID,
    can_db,
    gpio_wrapper,
    stepper_1_pin_id_config,
    USE_ENCODER,
    gpio_wrapper->get_gpio_num_for_pin_id(STEPPER_1_ENCODER_A_PIN_ID),
    gpio_wrapper->get_gpio_num_for_pin_id(STEPPER_1_ENCODER_B_PIN_ID),
    STEPPER_MOTOR_1_ADDRESS,
    endstop_switch,
    std::nullopt);
  e_drawer_0->init();

  drawers.push_back(e_drawer_0);

  debug_println("Finished setup()!");

  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(task_1_loop, /* Task function. */
                          "Task1",     /* name of task. */
                          10000,       /* Stack size of task */
                          NULL,        /* parameter of the task */
                          1,           /* priority of the task */
                          &Task1,      /* Task handle to keep track of created task */
                          0);          /* pin task to core 0 */

  delay(100);

  xTaskCreatePinnedToCore(task_2_loop, /* Task function. */
                          "Task2",     /* name of task. */
                          10000,       /* Stack size of task */
                          NULL,        /* parameter of the task */
                          1,           /* priority of the task */
                          &Task2,      /* Task handle to keep track of created task */
                          1);          /* pin task to core 1 */
}

void loop()
{
}
