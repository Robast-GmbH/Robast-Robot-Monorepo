

#ifndef ROBAST_DRAWER_MANAGER_DRAWER_MANAGER_HPP_
#define ROBAST_DRAWER_MANAGER__DRAWER_MANAGER_HPP_

#include <memory>
#include <stdio.h>
#include <string.h>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robast_hardware_nodes_msgs/srv/request_drawer.hpp"


namespace robast_drawer_manager
{



class DrawerManager : public rclcpp::Node
{
public:


  DrawerManager();
 
  ~DrawerManager();


};

}  // namespace robast_drawer_manager

#endif  // ROBAST_DRAWER_MANAGER__DRAWER_MANAGER_HPP_