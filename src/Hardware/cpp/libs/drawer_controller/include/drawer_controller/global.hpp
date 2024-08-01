#ifndef DRAWER_CONTROLLER_GLOBAL_HPP
#define DRAWER_CONTROLLER_GLOBAL_HPP

#include <memory>

#include "can_toolbox/can_controller.hpp"
#include "debug/debug.hpp"
#include "drawer/electrical_drawer.hpp"
#include "gpio/gpio_wrapper_pca9535.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "led/led_strip.hpp"
#include "motor/motor_monitor_configs.hpp"
#include "peripherals/gpio_defines.hpp"
#include "switch/switch.hpp"
#include "utils/config_manager.hpp"
#include "utils/data_mapper.hpp"
#include "utils/queue.hpp"

#define SWITCH_PRESSED_THRESHOLD 0.9
#define SWITCH_WEIGHT_NEW_VALUES 0.2

#define STEPPER_MOTOR_1_ADDRESS 0x00

TaskHandle_t Task1;
TaskHandle_t Task2;

SemaphoreHandle_t can_queue_mutex = NULL;

using drawer_ptr = std::shared_ptr<drawer_controller::IDrawer>;

std::shared_ptr<robast_can_msgs::CanDb> can_db;

std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper;

std::shared_ptr<drawer_controller::ElectricalDrawerLock> drawer_lock;

std::shared_ptr<drawer_controller::ElectricalDrawer> drawer;

std::shared_ptr<drawer_controller::ElectricalDrawerConfigs> drawer_configs;
std::shared_ptr<drawer_controller::EncoderConfigs> encoder_configs;
std::shared_ptr<drawer_controller::MotorConfigs> motor_configs;
std::shared_ptr<drawer_controller::MotorMonitorConfigs> motor_monitor_configs;

stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
  .stepper_enn_tmc2209_pin_id = STEPPER_1_ENN_TMC2209_PIN_ID,
  .stepper_stdby_tmc2209_pin_id = STEPPER_1_STDBY_TMC2209_PIN_ID,
  .stepper_spread_pin_id = STEPPER_1_SPREAD_PIN_ID,
  .stepper_dir_pin_id = STEPPER_1_DIR_PIN_ID,
  .stepper_diag_pin_id = STEPPER_1_DIAG_PIN_ID,
  .stepper_index_pin_id = STEPPER_1_INDEX_PIN_ID,
  .stepper_step_pin_id = STEPPER_1_STEP_PIN_ID};

std::shared_ptr<drawer_controller::Switch> endstop_switch;

std::unique_ptr<drawer_controller::DataMapper> data_mapper;

std::unique_ptr<drawer_controller::ConfigManager> config_manager;

std::unique_ptr<drawer_controller::CanController> can_controller;

// shared resource, so we need a mutex for this
std::unique_ptr<drawer_controller::Queue<robast_can_msgs::CanMessage>> can_msg_queue;

void receive_can_msg_task_loop(void* pvParameters)
{
  for (;;)
  {
    std::optional<robast_can_msgs::CanMessage> received_message = can_controller->handle_receiving_can_msg();
    if (received_message.has_value())
    {
      if (xSemaphoreTake(can_queue_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
      {
        debug_println("[Main]: Received CAN message and adding it to the queue.");
        can_msg_queue->add_element_to_queue(received_message.value());
        xSemaphoreGive(can_queue_mutex);
      }
      else
      {
        Serial.println("[Main]: Error: Could not take the mutex. This should not occur.");
      }
    }
  }
}

#endif   // DRAWER_CONTROLLER_GLOBAL_HPP