#include "robast_drawer_manager/drawer_manager.hpp"

namespace robast
{
  DrawerManager::DrawerManager(): Node("drawer_manager_Node")
  {
    RCLCPP_INFO(this->get_logger(), "Creating");
    this->drawers_info_server = this->create_service<ShelfSetupInfo>("Get_module_setup", bind(&DrawerManager::get_shelf_setup,this, placeholders::_1, placeholders::_2));
    authenticate_user_client = rclcpp_action::create_client<AuthenticateUser>(this,"/authenticate_user");
    open_drawers_client = rclcpp_action::create_client<DrawerUserAccess>(this, "/control_drawer");
    
    this->access_drawer_service = rclcpp_action::create_server<DrawerInteraction>(
    this,
    "drawerInteraction",
    bind(&DrawerManager::drawer_access_goal_callback, this, placeholders::_1, placeholders::_2),
    bind(&DrawerManager::drawer_access_cancel_callback, this, placeholders::_1),
    bind(&DrawerManager::drawer_access_accepted_callback, this, placeholders::_1)
    );

  }


  void DrawerManager::get_shelf_setup(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    
    drawers_info_client = this->create_client<ShelfSetupInfo>("/shelf_setup_info");

    auto clientRequest = std::make_shared<ShelfSetupInfo::Request>();
    auto service_responce_callback = [this, response]( rclcpp::Client<ShelfSetupInfo>::SharedFuture future) { response->drawers= future.get()->drawers;};
    auto result = drawers_info_client->async_send_request(request,service_responce_callback);
    
    // Wait for the result.
    while (!drawers_info_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return ;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service still running");
    }
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
    RCLCPP_INFO(this->get_logger(), "authentication_result_callback %s", goal_handle->get_goal().get()->task.ticket.load_key);
    std::thread{std::bind(&DrawerManager::check_drawer_permission, this, goal_handle)}.detach();
  }


  void DrawerManager::check_drawer_permission(const std::shared_ptr<GoalHandleDrawerInteraction> goal_handle)
  {
   
    const auto goal = goal_handle->get_goal();
    
    // NFC Reader
    RCLCPP_INFO(this->get_logger(), "check_drawer_permission %s", goal_handle->get_goal().get()->task.ticket.load_key);
      
    auto authentication_request = AuthenticateUser::Goal();
    authentication_request.permission_keys =  (goal.get()->loading ? goal.get()->task.ticket.load_key:goal.get()->task.ticket.drop_of_key);
    
    auto send_goal_options = rclcpp_action::Client<AuthenticateUser>::SendGoalOptions();
    send_goal_options.goal_response_callback = bind(&DrawerManager::authentication_goal_response_callback, this, placeholders::_1);
    send_goal_options.feedback_callback = bind(&DrawerManager::authentication_feedback_callback, this, placeholders::_1, placeholders:: _2);
    send_goal_options.result_callback = bind(&DrawerManager::authentication_result_callback, this, placeholders::_1, goal_handle );
 
    this->authenticate_user_client->async_send_goal(authentication_request, send_goal_options);
  }

  void DrawerManager::authentication_goal_response_callback( const GoalHandleAuthenticateUser::SharedPtr & goal_handle)
  {
    if (!goal_handle) 
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
  
  void DrawerManager::authentication_feedback_callback( GoalHandleAuthenticateUser::SharedPtr, const std::shared_ptr<const AuthenticateUser::Feedback> feedback)
  {
    if(feedback.get()->reader_status.is_active==true)
    {
      RCLCPP_INFO(this->get_logger(), "UserInterface:: Bitte halten sie ihren transponder and das Lesegerät?");
    }
    else if(feedback.get()->reader_status.is_completted== true)
    {
      RCLCPP_INFO(this->get_logger(), "UserInterface:: Danke fürs Authentifiziehren?");
    }
    else if(feedback.get()->reader_status.is_error== true)
    {
      RCLCPP_INFO(this->get_logger(), "UserInterface:: Leider ist ein Fehler aufgetreten. Bitte warten sie. Hilfe ist unterwegs?");
    }  

    return;
  }
  
  void DrawerManager::authentication_result_callback(const GoalHandleAuthenticateUser::WrappedResult & result, const std::shared_ptr<GoalHandleDrawerInteraction> task_handle)
  {

       RCLCPP_INFO(this->get_logger(), "authentication_result_callback %s", task_handle->get_goal().get()->task.ticket.load_key[0].c_str());
      start_open_drawer_action(task_handle);
  }

  void DrawerManager::open_drawer_goal_response_callback( const GoalHandleDrawerUserAccess::SharedPtr & goal_handle)
  {
    if (!goal_handle) 
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void DrawerManager::open_drawer_feedback_callback(  GoalHandleDrawerUserAccess::SharedPtr, const std::shared_ptr<const DrawerUserAccess::Feedback> feedback)
  {
    if(!feedback->is_drawer_open)
    {
      this->door_timer->cancel();
    }

  }

  void DrawerManager::open_drawer_result_callback(const GoalHandleDrawerUserAccess::WrappedResult & result, const std::shared_ptr<GoalHandleDrawerInteraction> task_handle )
  {

     RCLCPP_INFO(this->get_logger(), "UserInterface:: Wollen sie die Schublade noch mal öffnen?");
     if(false)
     {
       start_open_drawer_action(task_handle);
     }
     /*else 
     {
      auto responce = std::shared_ptr<robast_ros2_msgs::action::DrawerInteraction::Result>(); 
      responce->sucessful= true;
      task_handle->succeed(responce);

     }*/

  }
  void DrawerManager::start_open_drawer_action(const std::shared_ptr<GoalHandleDrawerInteraction> task_handle)
  {
    auto open_drawer_request = DrawerUserAccess ::Goal();
    open_drawer_request.drawer= task_handle->get_goal()->task.drawer;
    open_drawer_request.state=6;
    
    auto send_goal_options = rclcpp_action::Client<DrawerUserAccess>::SendGoalOptions();
    send_goal_options.goal_response_callback = bind(&DrawerManager::open_drawer_goal_response_callback, this, placeholders::_1);
    send_goal_options.feedback_callback = bind(&DrawerManager::open_drawer_feedback_callback, this, placeholders::_1, placeholders:: _2);
    send_goal_options.result_callback = bind(&DrawerManager::open_drawer_result_callback, this, placeholders::_1, task_handle);
    RCLCPP_INFO(this->get_logger(), " open drawer controller");
    this->open_drawers_client->async_send_goal(open_drawer_request, send_goal_options);

  }

  void DrawerManager::remind_user_to_close_drawer()
  {
    RCLCPP_INFO(this->get_logger(), "UserInterface::Bitte schließen sie die Schublade nachdem sie alles entnommen haben.");
  }

} // namespace robast