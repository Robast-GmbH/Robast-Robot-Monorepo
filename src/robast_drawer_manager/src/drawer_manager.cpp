#include <inttypes.h>
#include <memory>

#include "robast_drawer_manager/drawer_manager.hpp"

namespace robast_drawer_manager
{


  DrawerManager::DrawerManager(): Node("drawer_manager_Node")
  {
    RCLCPP_INFO(get_logger(), "Creating");
  }

  DrawerManager::~DrawerManager()
  {
    RCLCPP_INFO(get_logger(), "Destroying");
  }


} // namespace robast_drawer_manager