#ifndef DRAWER_CONTROLLER_DRAWER_HPP
#define DRAWER_CONTROLLER_DRAWER_HPP

#include <memory>
#include <optional>

#include "can/can_db.hpp"
#include "can/can_utils.hpp"
#include "interfaces/i_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "peripherals/electrical_lock.hpp"

namespace drawer_controller
{
  class Drawer : public IDrawer
  {
   public:
    Drawer(uint32_t module_id,
           uint8_t id,
           std::shared_ptr<robast_can_msgs::CanDb> can_db,
           std::shared_ptr<IGpioWrapper> gpio_wrapper);

    void init_electrical_lock(uint8_t pwr_open_lock_pin_id,
                              uint8_t pwr_close_lock_pin_id,
                              uint8_t sensor_lock_pin_id,
                              uint8_t sensor_drawer_closed_pin_id) override;

    void handle_electrical_lock_control() override;

    void can_in(robast_can_msgs::CanMessage msg) override;

    std::optional<robast_can_msgs::CanMessage> can_out() override;

    void update_state() override;

   private:
    uint32_t _module_id;
    uint8_t _id;
    std::shared_ptr<IGpioWrapper> _gpio_wrapper;
    std::unique_ptr<ElectricalLock> _electrical_lock;
    std::unique_ptr<CanUtils> _can_utils;

    bool _drawer_open_feedback_can_msg_sent = false;

    void handle_drawer_just_opened() override;

    void handle_drawer_just_closed() override;

    void debug_prints_drawer_lock(robast_can_msgs::CanMessage &can_message);
  };
}   // namespace drawer_controller
#endif
