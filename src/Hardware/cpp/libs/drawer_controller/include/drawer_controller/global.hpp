#ifndef DRAWER_CONTROLLER_GLOBAL_HPP
#define DRAWER_CONTROLLER_GLOBAL_HPP

#include <memory>

#include "debug/debug.hpp"
#include "drawer_controller/task_params.hpp"
#include "led/led_strip.hpp"
#include "peripherals/gpio_defines.hpp"

constexpr float SWITCH_PRESSED_THRESHOLD = 0.9;
constexpr float SWITCH_WEIGHT_NEW_VALUES = 0.2;

constexpr uint8_t STEPPER_MOTOR_1_ADDRESS = 0x00;

TaskHandle_t Task1;
TaskHandle_t Task2;

SemaphoreHandle_t can_queue_mutex = NULL;

using drawer_ptr = std::shared_ptr<interfaces::IDrawer>;

std::shared_ptr<global_params::TaskParams> task_params;

std::shared_ptr<global_params::TaskParamsReceiveCanMsgs> task_params_receive_can_msgs;

void receive_can_msg_task_loop(void* pvParameters)
{
  global_params::TaskParamsReceiveCanMsgs* task_params = (global_params::TaskParamsReceiveCanMsgs*) pvParameters;

  for (;;)
  {
    std::optional<robast_can_msgs::CanMessage> received_message =
      task_params->can_controller->handle_receiving_can_msg();
    if (received_message.has_value())
    {
      if (xSemaphoreTake(can_queue_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
      {
        debug_println("[Main]: Received CAN message and adding it to the queue.");
        task_params->can_msg_queue->enqueue(received_message.value());
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