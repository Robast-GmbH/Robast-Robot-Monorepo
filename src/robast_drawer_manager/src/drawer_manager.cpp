#include "robast_drawer_manager/drawer_manager.hpp"

namespace robast
{
  DrawerManager::DrawerManager(): Node("drawer_manager_Node")
  {
    RCLCPP_INFO(this->get_logger(), "Creating");
    this->drawers_info_server = this->create_service<ShelfSetupInfo>("get_module_setup", bind(&DrawerManager::get_shelf_setup,this, placeholders::_1, placeholders::_2));
    authenticate_user_client = rclcpp_action::create_client<AuthenticateUser>(this,"authenticate_user");
    open_drawers_client = rclcpp_action::create_client<DrawerUserAccess>(this, "control_drawer");
    
    this->drawer_interaction_server = rclcpp_action::create_server<DrawerInteraction>(
      this,
      "drawer_interaction",
      bind(&DrawerManager::drawer_interaction_goal_callback, this, placeholders::_1, placeholders::_2),
      bind(&DrawerManager::drawer_interaction_cancel_callback, this, placeholders::_1),
      bind(&DrawerManager::drawer_interaction_accepted_callback, this, placeholders::_1));
  }

  void DrawerManager::get_shelf_setup(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    
    drawers_info_client = this->create_client<ShelfSetupInfo>("shelf_setup_info");

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

  rclcpp_action::GoalResponse DrawerManager::drawer_interaction_goal_callback( const rclcpp_action::GoalUUID & uuid, shared_ptr<const DrawerInteraction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DrawerManager::drawer_interaction_cancel_callback(const shared_ptr<GoalHandleDrawerInteraction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DrawerManager::drawer_interaction_accepted_callback(const shared_ptr<GoalHandleDrawerInteraction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "open drawer"); //DEBUGGING
    RCLCPP_INFO(this->get_logger(), "authentication_result_callback %s", goal_handle->get_goal().get()->task.ticket.load_key); //DEBUGGING
    //TODO: Implement Unload
    std::thread{std::bind(&DrawerManager::handle_drawer_interaction, this, goal_handle)}.detach();
  }

  void DrawerManager::handle_drawer_interaction(const std::shared_ptr<GoalHandleDrawerInteraction> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    uint8_t state = 1;
    switch (state)
    {
    case 1:
      check_permission(goal->loading, goal->task->ticket->);

    case 2:
      handle_drawer_user_access();

    case 3:
      ask_user_for_reopening_drawer();
    
    default:
      break;
    }

    auto response = std::shared_ptr<robast_ros2_msgs::action::DrawerInteraction::Result>();
    response->error_message = "";
    goal_handle->succeed(response);

    // this->check_permission_and_trigger_drawer_user_access();
  }

  void DrawerManager::check_permission(bool loading, std::vector<string> load_keys, std::vector<string> drop_of_keys)
  {    
    // NFC Reader
    RCLCPP_INFO(this->get_logger(), "check_drawer_permission for load_keys[0]: %s", load_keys[0]); //DEBUGGING
      
    auto authentication_request = AuthenticateUser::Goal();
    authentication_request.permission_keys =  loading ? load_keys : drop_of_keys;
    
    auto send_goal_options = rclcpp_action::Client<AuthenticateUser>::SendGoalOptions();
    send_goal_options.goal_response_callback = bind(&DrawerManager::authentication_goal_response_callback, this, placeholders::_1);
    send_goal_options.feedback_callback = bind(&DrawerManager::authentication_feedback_callback, this, placeholders::_1, placeholders:: _2);
    // in the authentication_result_callback the drawer user access will be triggered
    send_goal_options.result_callback = bind(&DrawerManager::authentication_result_callback, this, placeholders::_1); //TODO: remove goal_handle_drawer_interaction
 
    this->authenticate_user_client->async_send_goal(authentication_request, send_goal_options);
  }

  void DrawerManager::check_permission_and_trigger_drawer_user_access()
  {
    const auto goal = this->goal_handle_drawer_interaction->get_goal();
    
    // NFC Reader
    RCLCPP_INFO(this->get_logger(), "check_drawer_permission %s", this->goal_handle_drawer_interaction->get_goal().get()->task.ticket.load_key); //DEBUGGING
      
    auto authentication_request = AuthenticateUser::Goal();
    authentication_request.permission_keys =  goal->loading ? goal->task.ticket.load_key : goal->task.ticket.drop_of_key;
    
    auto send_goal_options = rclcpp_action::Client<AuthenticateUser>::SendGoalOptions();
    send_goal_options.goal_response_callback = bind(&DrawerManager::authentication_goal_response_callback, this, placeholders::_1);
    send_goal_options.feedback_callback = bind(&DrawerManager::authentication_feedback_callback, this, placeholders::_1, placeholders:: _2);
    // in the authentication_result_callback the drawer user access will be triggered
    send_goal_options.result_callback = bind(&DrawerManager::authentication_result_callback, this, placeholders::_1, this->goal_handle_drawer_interaction); //TODO: remove goal_handle_drawer_interaction
 
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
  
  //TODO: parameter name missing for GoalHandleDrawerUserAccess::SharedPtr 
  void DrawerManager::authentication_feedback_callback(GoalHandleAuthenticateUser::SharedPtr, const std::shared_ptr<const AuthenticateUser::Feedback> feedback)
  {
    if(feedback.get()->reader_status.is_reading)
    {
      RCLCPP_INFO(this->get_logger(), "UserInterface:: Bitte halten sie ihren transponder and das Lesegerät?");
    }
    else if(feedback.get()->reader_status.is_completed)
    {
      RCLCPP_INFO(this->get_logger(), "UserInterface:: Danke fürs Authentifiziehren?");
    }

    return;
  }
  
  void DrawerManager::authentication_result_callback(const GoalHandleAuthenticateUser::WrappedResult & result, const std::shared_ptr<GoalHandleDrawerInteraction> task_handle)
  {
    RCLCPP_INFO(this->get_logger(), "authentication_result_callback %s", task_handle->get_goal().get()->task.ticket.load_key[0].c_str());
    //TODO: Check if action result is successful 
    this->start_open_drawer_action(task_handle);
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

  //TODO: parameter name missing for GoalHandleDrawerUserAccess::SharedPtr 
  void DrawerManager::open_drawer_feedback_callback(GoalHandleDrawerUserAccess::SharedPtr , const std::shared_ptr<const DrawerUserAccess::Feedback> feedback)
  {
    if(!feedback->is_drawer_open)
    {
      this->drawer_open_alert_timer->cancel();
      //TODO: Start this timer somewhere and handle timer alter
    }
  }

  void DrawerManager::open_drawer_result_callback(const GoalHandleDrawerUserAccess::WrappedResult & result, const std::shared_ptr<GoalHandleDrawerInteraction> task_handle )
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "DrawerUserAccess Goal was aborted");
        //TODO: What do we do if DrawerUserAccess is not successful? Send message to staff?
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "DrawerUserAccess Goal was canceled");
        //TODO: What do we do if DrawerUserAccess is not successful? Send message to staff?
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code for DrawerUserAccess");
        //TODO: What do we do if DrawerUserAccess is not successful? Send message to staff?
        return;
    }

    RCLCPP_INFO(this->get_logger(), "UserInterface:: Wollen sie die Schublade noch mal öffnen?");
    //TODO: Implement interaction with user interface
    if(false)
    {
      this->start_open_drawer_action(this->goal_handle_drawer_interaction);
    }
    else 
     {
      //TODO: Fix this
      // auto response = std::shared_ptr<robast_ros2_msgs::action::DrawerInteraction::Result>();
      // response->error_message = "";
      // this->goal_handle_drawer_interaction->succeed(response);
     }
  }

  void DrawerManager::start_open_drawer_action(const std::shared_ptr<GoalHandleDrawerInteraction> task_handle)
  {
    auto open_drawer_request = DrawerUserAccess::Goal();
    open_drawer_request.drawer_address = this->goal_handle_drawer_interaction->get_goal()->task.drawer_address;
    open_drawer_request.state = 6;
    
    auto send_goal_options = rclcpp_action::Client<DrawerUserAccess>::SendGoalOptions();
    send_goal_options.goal_response_callback = bind(&DrawerManager::open_drawer_goal_response_callback, this, placeholders::_1);
    send_goal_options.feedback_callback = bind(&DrawerManager::open_drawer_feedback_callback, this, placeholders::_1, placeholders:: _2);
    send_goal_options.result_callback = bind(&DrawerManager::open_drawer_result_callback, this, placeholders::_1, this->goal_handle_drawer_interaction);
    RCLCPP_INFO(this->get_logger(), " open drawer controller");
    this->open_drawers_client->async_send_goal(open_drawer_request, send_goal_options);
  }

  void DrawerManager::remind_user_to_close_drawer()
  {
    RCLCPP_INFO(this->get_logger(), "UserInterface::Bitte schließen sie die Schublade nachdem sie alles entnommen haben.");
  }

} // namespace robast