#ifndef DRAWER_E_DRAWER_CONFIG_MANAGER_HPP
#define DRAWER_E_DRAWER_CONFIG_MANAGER_HPP

#include <bit>
#include <memory>

#include "drawer/electrical_drawer_config.hpp"
#include "module_config/module_config_defines.hpp"
#include "motor/enconder_config.hpp"
#include "motor/motor_config.hpp"
#include "motor/motor_monitor_config.hpp"
#include "utils/base_config_manager.hpp"

namespace drawer
{
  class EDrawerConfigManager : public utils::BaseConfigManager
  {
  public:
    EDrawerConfigManager();

    bool set_config(const uint8_t config_id, const uint32_t config_value) override;

    void print_all_configs() const override;

    std::shared_ptr<drawer::ElectricalDrawerConfig> get_drawer_config() const;
    std::shared_ptr<motor::EncoderConfig> get_encoder_config() const;
    std::shared_ptr<motor::MotorConfig> get_motor_config() const;
    std::shared_ptr<motor::MotorMonitorConfig> get_motor_monitor_config() const;

  protected:
    bool set_e_drawer_config(const uint8_t config_id, const uint32_t config_value);

    void print_e_drawer_configs() const;

  private:
    const std::shared_ptr<drawer::ElectricalDrawerConfig> _drawer_config =
        std::make_shared<drawer::ElectricalDrawerConfig>();
    const std::shared_ptr<motor::EncoderConfig> _encoder_config = std::make_shared<motor::EncoderConfig>();
    const std::shared_ptr<motor::MotorConfig> _motor_config = std::make_shared<motor::MotorConfig>();
    const std::shared_ptr<motor::MotorMonitorConfig> _motor_monitor_config =
        std::make_shared<motor::MotorMonitorConfig>();

    void set_default_configs();

    void set_default_drawer_config();

    void set_default_encoder_config();

    void set_default_motor_config();

    void set_default_motor_monitor_config();
  };

} // namespace drawer

#endif // DRAWER_E_DRAWER_CONFIG_MANAGER_HPP
