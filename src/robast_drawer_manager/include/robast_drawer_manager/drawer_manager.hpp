

#ifndef ROBAST_DRAWER_MANAGER_DRAWER_MANAGER_HPP_
#define ROBAST_DRAWER_MANAGER__DRAWER_MANAGER_HPP_

#include <memory>
#include <stdio.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robast_ros2_msgs/srv/get_all_attached_drawers.hpp"
#include "robast_ros2_msgs/srv/handle_drawer.hpp"
#include "robast_ros2_msgs/action/drawer_user_access.hpp"
#include "robast_ros2_msgs/action/authenticate_user.hpp"


namespace robast_drawer_manager
{


  class DrawerManager : public rclcpp::Node
  {

    public: 
      using AllAttechedDrawers = robast_ros2_msgs::srv::GetAllAttachedDrawers;
      using HandleDrawer       = robast_ros2_msgs::srv::HandleDrawer;
      using DrawerUserAccess   = robast_ros2_msgs::action::DrawerUserAccess

      DrawerManager();
     // ~DrawerManager();
     

  private:

    rclcpp_action::Client<AuthenticateUser>::SharedPtr authenticate_user_client;
    rclcpp_action::Client<DrawerUserAccess>::SharedPtr open_drawers_client;

    rclcpp::Service<GetAllAttachedDrawers>::SharedPtr get_all_drawers_server;
    rclcpp::Service<GetAllAttachedDrawers>::SharedPtr open_drawer_server;

    void get_all_drawer(const std::shared_ptr<AllAttechedDrawers::Request> request, std::shared_ptr<AllAttechedDrawers::Response>      response);

    void open_drawer(const std::shared_ptr<HandleDrawer::Request> request,  std::shared_ptr<HandleDrawer::Response>      response);
  
    void remind_user_to_close_drawer();

    /*void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<DrawerUserAccess>::SharedPtr> future);
    void open_drawer_feedback_callback( rclcpp_action::ClientGoalHandle<DrawerUserAccess>::SharedPtr, const std::shared_ptr<const DrawerUserAccess::Feedback> feedback);
    void open_drawer_result_callback(const rclcpp_action::ClientGoalHandle<DrawerUserAccess>::WrappedResult & result)*/

};

}  // namespace robast_drawer_manager

#endif  // ROBAST_DRAWER_MANAGER__DRAWER_MANAGER_HPP_