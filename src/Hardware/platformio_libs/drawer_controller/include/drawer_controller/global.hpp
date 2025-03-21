#ifndef DRAWER_CONTROLLER_GLOBAL_HPP
#define DRAWER_CONTROLLER_GLOBAL_HPP

#include <memory>

#include "config/module_hardware_config.hpp"
#include "config/user_config.hpp"
#include "debug/debug.hpp"
#include "drawer/electrical_drawer.hpp"
#include "drawer/manual_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "led/led_strip.hpp"
#include "logging/rotating_file_handler.hpp"
#include "module_id/module_id.hpp"
#include "motor/motor_monitor_config.hpp"
#include "switch/switch.hpp"
#include "utils/can_message_converter.hpp"
#include "utils/queue.hpp"
#include "watchdog/heartbeat.hpp"
#include "drawer/motion_controller.hpp"

constexpr float SWITCH_PRESSED_THRESHOLD = 0.9;
constexpr float SWITCH_WEIGHT_NEW_VALUES = 0.25;

constexpr uint8_t STEPPER_MOTOR_1_ADDRESS = 0x00;

constexpr uint8_t MINIMAL_LOOP_TIME_IN_MS = 1;

TaskHandle_t Task1;
TaskHandle_t Task2;

SemaphoreHandle_t can_queue_mutex = NULL;

std::shared_ptr<robast_can_msgs::CanDb> can_db;

std::shared_ptr<can_toolbox::CanUtils> can_utils;

std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper;

std::shared_ptr<lock::ElectricalDrawerLock> drawer_lock;

std::shared_ptr<drawer::MotionController> motion_controller;

std::shared_ptr<interfaces::IDrawer> i_drawer;

std::shared_ptr<switch_lib::Switch> endstop_switch;

std::unique_ptr<utils::CanMessageConverter> can_message_converter;

std::shared_ptr<watchdog::Heartbeat> heartbeat;

std::shared_ptr<logging::RotatingFileHandler> rotating_file_logger;

// shared resource, so we need a mutex for this
std::unique_ptr<utils::Queue<robast_can_msgs::CanMessage>> can_msg_queue;

#endif // DRAWER_CONTROLLER_GLOBAL_HPP
