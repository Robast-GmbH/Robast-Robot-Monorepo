#include "robast_drawer_manager/drawer_manager.hpp"

namespace robast_drawer_manager
{
  DrawerManager::DrawerManager(): Node("drawer_manager_Node")
  {
    RCLCPP_INFO(this->get_logger(), "Creating");
    this->drawers_info_server = this->create_service<ShelfSetupInfo>("Get_module_setup", bind(&DrawerManager::get_shelf_setup,this, placeholders::_1, placeholders::_2));
    //this->open_drawer_server = this->create_service<HandleDrawer>("open_drawer_drawer_management", bind(&DrawerManager::open_drawer,this, placeholders::_1, placeholders::_2));
    
    this->access_drawer_service = rclcpp_action::create_server<DrawerInteraction>(
    this,
    "drawerInteraction",
    bind(&DrawerManager::drawer_access_goal_callback, this, placeholders::_1, placeholders::_2),
    bind(&DrawerManager::drawer_access_cancel_callback, this, placeholders::_1),
    bind(&DrawerManager::drawer_access_accepted_callback, this, placeholders::_1)
    );

    this->authenticate_user_client = rclcpp_action::create_client<robast_ros2_msgs::action::AuthenticateUser>(this,"authenticate_drawer_user");
    this->open_drawers_client = rclcpp_action::create_client<DrawerUserAccess>(this,"open_drawer_drawer");
  }


  void DrawerManager::get_shelf_setup(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    //# request from the Drawer Hardware Node the setup modules used(just the modules which are empty)     

    //response->drawers;
  }

  rclcpp_action::GoalResponse DrawerManager::drawer_access_goal_callback( const rclcpp_action::GoalUUID & uuid, shared_ptr<const DrawerInteraction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DrawerManager::drawer_access_cancel_callback(const shared_ptr<GoalHandleDrawerInteraction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DrawerManager::drawer_access_accepted_callback(const shared_ptr<GoalHandleDrawerInteraction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "open drawer");
    std::thread{std::bind(&DrawerManager::open_drawer, this, placeholders::_1), goal_handle}.detach();
  }


  void DrawerManager::open_drawer(const std::shared_ptr<GoalHandleDrawerInteraction> goal_handle)
  {
    /*// NFC Reader
    auto authentication_request = std::make_shared<robast_ros2_msgs::srv::AuthenticateUser::Request>();
    authentication_request.permission_keys = request.permission_keys;

    //auto result =  this->authenticate_user_client ->async_send_request(authentication_request);
    //RCLCPP_INFO(this->get_logger(), "The reader is active please authenticate yourselve at the reader.");
    if (!rclcpp::spin_until_future_complete(this, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the NFC module");
      return; 
    }

    if(!result.get()->sucessful)
    {
      RCLCPP_ERROR(this->get_logger(), result.get()->error_message);
      return; 
    }

    //result.get()->permission_key_used //TODO  use 
   
    //öffne schublade
  
    if (!this->open_drawers_client->wait_for_action_server()) 
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the Drawergate");
      rclcpp::shutdown();
    }

    auto target_drawer_msg = DrawerUserAccess::Goal();
    target_drawer_msg.order = request->drawer_id;

    auto send_goal_options = rclcpp_action::Client<DrawerUserAccess>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&DrawerManager::open_drawer_feedback_callback, this, placeholders::_1, placeholders::_2);
    auto result = open_drawers_client->async_send_goal(target_drawer_msg, send_goal_options);

    // Schublade offen im Frontend anzeigen
    RCLCPP_INFO(this->get_logger(), "The drawer is open please close the drawer after you are done.");
      
    // nach zeit intervall //bitte schließen imfrontend anzeigen
    this->door_timer = this->create_wall_timer(  std::chrono::milliseconds(500), std::bind(&DrawerManager::remind_user_to_close_drawer, this));
    
    if (!rclcpp::spin_until_future_complete(this, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the drawer gate");
      return; 
    }
    
    //  wenn die Schublade  geschlossen ist action beenden */
  }



  /*void DrawerManager::open_drawer_feedback_callback( rclcpp_action::ClientGoalHandle<DrawerUserAccess>::SharedPtr, const std::shared_ptr<const DrawerUserAccess::Feedback> feedback)
  {
    if(feedback->is_drawer_open)
      { 
        this->door_timer = this->create_wall_timer(  std::chrono::milliseconds(500), std::bind(&DrawerManager::remind_user_to_close_drawer, this));
      }
      else 
      {
       
      }
  }
*/
 /* void DrawerManager::remind_user_to_close_drawer()
  {
    RCLCPP_INFO(this->get_logger(), "Please close the drawer when you are done.");
  }*/

} // namespace robast_drawer_manager