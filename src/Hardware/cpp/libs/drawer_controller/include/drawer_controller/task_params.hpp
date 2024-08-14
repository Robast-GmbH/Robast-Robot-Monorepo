#ifndef UTILS_TASK_PARAMS_HPP
#define UTILS_TASK_PARAMS_HPP

#include "can_toolbox/can_controller.hpp"
#include "drawer/electrical_drawer.hpp"
#include "gpio/gpio_wrapper_pca9535.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "motor/motor_monitor_config.hpp"
#include "switch/switch.hpp"
#include "utils/can_message_converter.hpp"
#include "utils/config_manager.hpp"
#include "utils/queue.hpp"

namespace global_params
{
  struct TaskParams
  {
    std::shared_ptr<robast_can_msgs::CanDb> can_db;
    std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper;
    std::shared_ptr<lock::ElectricalDrawerLock> drawer_lock;
    std::shared_ptr<drawer::ElectricalDrawer> e_drawer;

    std::shared_ptr<drawer::ElectricalDrawerConfig> drawer_config;
    std::shared_ptr<motor::EncoderConfig> encoder_config;
    std::shared_ptr<motor::MotorConfig> motor_config;
    std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config;

    std::shared_ptr<utils::ConfigManager> config_manager;

    std::shared_ptr<switch_lib::Switch> endstop_switch;

    std::shared_ptr<utils::CanMessageConverter> can_message_converter;

    std::shared_ptr<can_toolbox::CanController> can_controller;

    // shared resource, so we need a mutex for this
    std::shared_ptr<utils::Queue<robast_can_msgs::CanMessage>> can_msg_queue;

    TaskParams(const std::shared_ptr<robast_can_msgs::CanDb> can_db,
               const std::shared_ptr<interfaces::IGpioWrapper> gpio_wrapper,
               const std::shared_ptr<lock::ElectricalDrawerLock> drawer_lock,
               const std::shared_ptr<drawer::ElectricalDrawer> e_drawer,
               const std::shared_ptr<drawer::ElectricalDrawerConfig> drawer_config,
               const std::shared_ptr<motor::EncoderConfig> encoder_config,
               const std::shared_ptr<motor::MotorConfig> motor_config,
               const std::shared_ptr<motor::MotorMonitorConfig> motor_monitor_config,
               const std::shared_ptr<utils::ConfigManager> config_manager,
               const std::shared_ptr<switch_lib::Switch> endstop_switch,
               const std::shared_ptr<utils::CanMessageConverter> can_message_converter,
               const std::shared_ptr<can_toolbox::CanController> can_controller,
               const std::shared_ptr<utils::Queue<robast_can_msgs::CanMessage>> can_msg_queue)
        : can_db(can_db),
          gpio_wrapper(gpio_wrapper),
          drawer_lock(drawer_lock),
          e_drawer(e_drawer),
          drawer_config(drawer_config),
          encoder_config(encoder_config),
          motor_config(motor_config),
          motor_monitor_config(motor_monitor_config),
          config_manager(config_manager),
          endstop_switch(endstop_switch),
          can_message_converter(can_message_converter),
          can_controller(can_controller),
          can_msg_queue(can_msg_queue)
    {
    }
  };

  struct TaskParamsReceiveCanMsgs
  {
    const std::shared_ptr<can_toolbox::CanController> can_controller;
    const std::shared_ptr<utils::Queue<robast_can_msgs::CanMessage>> can_msg_queue;

    TaskParamsReceiveCanMsgs(const std::shared_ptr<can_toolbox::CanController> can_controller,
                             const std::shared_ptr<utils::Queue<robast_can_msgs::CanMessage>> can_msg_queue)
        : can_controller(can_controller), can_msg_queue(can_msg_queue)
    {
    }
  };

}   // namespace global_params

#endif   // UTILS_TASK_PARAMS_HPP