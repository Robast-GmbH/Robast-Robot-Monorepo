#ifndef TRAY_PARTIAL_DRAWER_CONFIG_MANAGER_HPP
#define TRAY_PARTIAL_DRAWER_CONFIG_MANAGER_HPP

#include <bit>
#include <memory>

#include "drawer/e_drawer_config_manager.hpp"
#include "tray/tray_manager_config.hpp"

namespace tray
{
  class PartialDrawerConfigManager : public drawer::EDrawerConfigManager
  {
  public:
    PartialDrawerConfigManager();

    bool set_config(const uint8_t config_id, const uint32_t config_value) override;

    void print_all_configs() const override;

    std::shared_ptr<tray::TrayManagerConfig> get_tray_manager_config() const;

  private:
    const std::shared_ptr<tray::TrayManagerConfig> _tray_manager_config = std::make_shared<tray::TrayManagerConfig>();

    void set_default_configs();

    void set_default_tray_manager_config();
  };

} // namespace tray

#endif // TRAY_PARTIAL_DRAWER_CONFIG_MANAGER_HPP
