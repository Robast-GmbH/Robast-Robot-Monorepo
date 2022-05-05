

#ifndef ROBAST_DRAWER_MANAGER_DRAWER_MANAGER_HPP_
#define ROBAST_DRAWER_MANAGER__DRAWER_MANAGER_HPP_

#include <memory>
#include <stdio.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robast_msgs/srv/get_all_attached_drawers.hpp"
#include "robast_msgs/action/drawer_user_access.hpp"


namespace robast_drawer_manager
{


class DrawerManager : public rclcpp::Node
{
public:


  DrawerManager();
 
  ~DrawerManager();

private:

   rclcpp_action::Client<DrawerUserAccess>::SharedPtr open_drawers_client;
   rclcpp::Service<GetAllAttachedDrawers>::SharedPtr get_all_drawers_server;
   rclcpp::Service<GetAllAttachedDrawers>::SharedPtr open_drawer_server;
 
   void get_all_drawer(const std::shared_ptr<robast_msgs::srv::GetAllAttachedDrawers::Request> request,
                      std::shared_ptr<robast_msgs::srv::GetAllAttachedDrawers::Response>      response);

  void open_drawer(const std::shared_ptr<robast_msgs::srv::HandleDrawer::Request> request,
                        std::shared_ptr<robast_msgs::srv::HandleDrawer::Response>      response)

};

}  // namespace robast_drawer_manager

#endif  // ROBAST_DRAWER_MANAGER__DRAWER_MANAGER_HPP_