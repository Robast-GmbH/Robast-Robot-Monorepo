#include "rclcpp/rclcpp.hpp"
#include "ra_nav_door_bell/srv/get_door_bell_pose.hpp"
#include <inttypes.h>
#include <memory>

namespace ra_nav_door_bell
{

DoorBellFinder::DoorBellFinder()
: nav2_util::LifecycleNode("ra_nav_door_bell", "", true),
{
  RCLCPP_INFO(get_logger(), "Creating");


}

//TODO: on_configure
//TODO: on_activate
//TODO: on_deactivate
//TODO: on_cleanup
//TODO: on_shutdown

} // namespace ra_nav_door_bell