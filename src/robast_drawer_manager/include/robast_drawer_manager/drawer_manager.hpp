

#ifndef ROBAST_DRAWER_MANAGER_DRAWER_MANAGER_HPP_
#define ROBAST_DRAWER_MANAGER__DRAWER_MANAGER_HPP_

#include <memory>
#include <stdio.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robast_ros2_msgs/srv/shelf_setup_info.hpp"

#include "robast_ros2_msgs/action/drawer_interaction.hpp"
#include "robast_ros2_msgs/action/drawer_user_access.hpp"
#include "robast_ros2_msgs/action/authenticate_user.hpp"
 using namespace std;

namespace robast
{


  class DrawerManager : public rclcpp::Node
  {

    public: 
      using ShelfSetupInfo                = robast_ros2_msgs::srv::ShelfSetupInfo;  // to get the types of modules in the Robot
      
      using DrawerInteraction             = robast_ros2_msgs::action::DrawerInteraction;    // the Service provideded By this Node to perform 
      using GoalHandleDrawerInteraction   = rclcpp_action::ServerGoalHandle<DrawerInteraction>;
      
      using AuthenticateUser              = robast_ros2_msgs::action::AuthenticateUser;  //interaction with the 
      using GoalHandleAuthenticateUser    = rclcpp_action::ClientGoalHandle<AuthenticateUser>;
      using AuthenticateUserResultHandle  = std::shared_future<std::shared_ptr<robast::DrawerManager::GoalHandleAuthenticateUser>>;
     
      using DrawerUserAccess              = robast_ros2_msgs::action::DrawerUserAccess; // interaction with the drawermodules 
      using GoalHandleDrawerUserAccess    = rclcpp_action::ClientGoalHandle<DrawerUserAccess >;
      using DrawerUserAccessResultHandle  = std::shared_future<std::shared_ptr<robast::DrawerManager::GoalHandleDrawerUserAccess>>;


      using DrawerAddress                 = robast_ros2_msgs::msg::DrawerAddress;

      DrawerManager();
     // ~DrawerManager();
     

  private:

    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor;
    
    rclcpp::TimerBase::SharedPtr drawer_open_alert_timer;
  
    rclcpp::Service<ShelfSetupInfo>::SharedPtr drawers_info_server;
    rclcpp::Client<ShelfSetupInfo>::SharedPtr drawers_info_client;

    rclcpp_action::Server<DrawerInteraction>::SharedPtr drawer_interaction_server;
    rclcpp_action::Client<AuthenticateUser>::SharedPtr authenticate_user_client;
    rclcpp_action::Client<DrawerUserAccess>::SharedPtr user_drawer_access_client;


    void get_shelf_setup(const shared_ptr<ShelfSetupInfo::Request> request, shared_ptr<ShelfSetupInfo::Response> response);

    rclcpp_action::GoalResponse drawer_interaction_goal_callback(const rclcpp_action::GoalUUID & uuid, shared_ptr<const DrawerInteraction::Goal> goal);
    rclcpp_action::CancelResponse drawer_interaction_cancel_callback(const shared_ptr<GoalHandleDrawerInteraction> goal_handle); 
    void drawer_interaction_accepted_callback(const shared_ptr<GoalHandleDrawerInteraction> goal_handle);

    void authentication_goal_response_callback( const GoalHandleAuthenticateUser::SharedPtr & goal_handle);
    void authentication_feedback_callback( GoalHandleAuthenticateUser::SharedPtr, const std::shared_ptr<const AuthenticateUser::Feedback> feedback);
   
    void open_drawer_goal_response_callback( const GoalHandleDrawerUserAccess::SharedPtr & goal_handle);
    void open_drawer_feedback_callback(  GoalHandleDrawerUserAccess::SharedPtr, const std::shared_ptr<const DrawerUserAccess::Feedback> feedback);

    void handle_drawer_interaction(const std::shared_ptr<GoalHandleDrawerInteraction> goal_handle);
    void drawer_interaction_state_machine(const std::shared_ptr<const robast_ros2_msgs::action::DrawerInteraction_Goal> goal, uint8_t state = 1);

    AuthenticateUserResultHandle request_user_authentication(bool loading, std::vector<string> load_keys, std::vector<string> drop_of_keys);
    string wait_for_user_authentication(AuthenticateUserResultHandle action_handle);
    DrawerUserAccessResultHandle request_drawer_user_access(DrawerAddress drawer_address);
    void wait_for_finished_drawer_user_access(DrawerUserAccessResultHandle drawer_user_access_action_handle);
    void ask_user_for_reopening_drawer(const std::shared_ptr<const robast_ros2_msgs::action::DrawerInteraction_Goal> goal);

    void open_drawer(const std::shared_ptr<GoalHandleDrawerInteraction> goal_handle);
    void remind_user_to_close_drawer();
    void start_open_drawer_action(const std::shared_ptr<GoalHandleDrawerInteraction> task_handle);
    
};

}  // namespace robast_drawer_manager

#endif  // ROBAST_DRAWER_MANAGER__DRAWER_MANAGER_HPP_