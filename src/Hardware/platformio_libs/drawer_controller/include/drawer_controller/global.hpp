#ifndef DRAWER_CONTROLLER_GLOBAL_HPP
#define DRAWER_CONTROLLER_GLOBAL_HPP

#include <memory>

#include "debug/debug.hpp"
#include "drawer/electrical_drawer.hpp"
#include "drawer/manual_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "led/led_strip.hpp"
#include "motor/motor_monitor_config.hpp"
#include "switch/switch.hpp"
#include "utils/can_message_converter.hpp"
#include "utils/config_manager.hpp"
#include "utils/queue.hpp"

constexpr float SWITCH_PRESSED_THRESHOLD = 0.9;
constexpr float SWITCH_WEIGHT_NEW_VALUES = 0.25;

constexpr uint8_t STEPPER_MOTOR_1_ADDRESS = 0x00;

TaskHandle_t Task1;
TaskHandle_t Task2;

SemaphoreHandle_t can_queue_mutex = NULL;

constexpr uint8_t MINIMAL_LOOP_TIME_IN_MS = 1;

std::shared_ptr<robast_can_msgs::CanDb> can_db;

std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper;

std::shared_ptr<lock::ElectricalDrawerLock> drawer_lock;

std::shared_ptr<interfaces::IDrawer> i_drawer;

std::shared_ptr<drawer::ElectricalDrawerConfig> drawer_config;
std::shared_ptr<motor::EncoderConfig> encoder_config;
std::shared_ptr<motor::MotorConfig> motor_config;
std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config;

std::shared_ptr<switch_lib::Switch> endstop_switch;

std::unique_ptr<utils::CanMessageConverter> can_message_converter;

std::unique_ptr<utils::ConfigManager> config_manager;

// shared resource, so we need a mutex for this
std::unique_ptr<utils::Queue<robast_can_msgs::CanMessage>> can_msg_queue;

#endif   // DRAWER_CONTROLLER_GLOBAL_HPP