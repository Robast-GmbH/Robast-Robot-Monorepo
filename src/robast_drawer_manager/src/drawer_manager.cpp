#include "robast_drawer_manager/drawer_manager.hpp"

namespace robast
{
  DrawerManager::DrawerManager(): Node("drawer_manager_Node")
  {
    RCLCPP_INFO(this->get_logger(), "Creating");

    callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor.add_callback_group(callback_group, this->get_node_base_interface());

    this->drawers_info_server = this->create_service<ShelfSetupInfo>("get_module_setup", bind(&DrawerManager::get_shelf_setup,this, placeholders::_1, placeholders::_2));
    this->drawers_info_client = this->create_client<ShelfSetupInfo>("shelf_setup_info", rmw_qos_profile_services_default, callback_group);
    this->authenticate_user_client = rclcpp_action::create_client<AuthenticateUser>(this,"authenticate_user");
    this->user_drawer_access_client = rclcpp_action::create_client<DrawerUserAccess>(this, "control_drawer");
    
    this->drawer_interaction_server = rclcpp_action::create_server<DrawerInteraction>(
      this,
      "drawer_interaction",
      bind(&DrawerManager::drawer_interaction_goal_callback, this, placeholders::_1, placeholders::_2),
      bind(&DrawerManager::drawer_interaction_cancel_callback, this, placeholders::_1),
      bind(&DrawerManager::drawer_interaction_accepted_callback, this, placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Finished Creating");
  }


  void DrawerManager::get_shelf_setup(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    // Wait for the service to be alive.
    while (!drawers_info_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return ;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service still running");
    }

    auto clientRequest = std::make_shared<ShelfSetupInfo::Request>();
    auto result_handle = drawers_info_client->async_send_request(request);

    const std::chrono::seconds timeout = std::chrono::seconds(5);
    if (callback_group_executor.spin_until_future_complete(result_handle, timeout) != rclcpp::FutureReturnCode::SUCCESS)
    {
      //TODO: Implement reasonable error handling
      RCLCPP_ERROR(this->get_logger(), "get_shelf_setup service client: async_send_request failed"); // DEBUGGING
    }

    response->drawers = result_handle.get()->drawers;

    RCLCPP_INFO(this->get_logger(), "get_shelf_setup finished"); // DEBUGGING
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
    RCLCPP_INFO(this->get_logger(), "authentication_result_callback %s", goal_handle->get_goal().get()->task.ticket.load_keys[0].c_str()); //DEBUGGING
    //TODO: Implement Unload
    std::thread{std::bind(&DrawerManager::handle_drawer_interaction, this, goal_handle)}.detach();
  }

  void DrawerManager::handle_drawer_interaction(const std::shared_ptr<GoalHandleDrawerInteraction> goal_handle)
  {
    const std::shared_ptr<const robast_ros2_msgs::action::DrawerInteraction_Goal> goal = goal_handle->get_goal();

    this->drawer_interaction_state_machine(goal);    
    auto response = std::make_shared<robast_ros2_msgs::action::DrawerInteraction::Result>();
    response->error_message = "";
    goal_handle->succeed(response);
  }


  void DrawerManager::drawer_interaction_state_machine(const std::shared_ptr<const robast_ros2_msgs::action::DrawerInteraction_Goal> goal, uint8_t state)
  {
    switch (state)
    {
      case 1:
      {
        // 1. step: Authenticate the User that should access the drawer
        DrawerManager::AuthenticateUserResultHandle user_id_action_handle = this->request_user_authentication(goal->loading, goal->task.ticket.load_keys, goal->task.ticket.drop_of_keys);
        if (this->wait_for_user_authentication(user_id_action_handle) == "")
        {
          return; //TODO: Handle case of error
        }       
      }

      case 2:
      {
        // 2. step: Start the drawer user access
        DrawerManager::DrawerUserAccessResultHandle drawer_user_access_action_handle = this->request_drawer_user_access(goal->task.drawer_address);
        this->wait_for_finished_drawer_user_access(drawer_user_access_action_handle);
      }
        
      case 3:
      {
        // 3. step: Ask user if he wants to reopen the drawer again
        this->ask_user_for_reopening_drawer(goal);
      }
      
      default:
        break;
    }
    return;
  }


  DrawerManager::AuthenticateUserResultHandle DrawerManager::request_user_authentication(bool loading, std::vector<string> load_keys, std::vector<string> drop_of_keys)
  {    
    // NFC Reader
    RCLCPP_INFO(this->get_logger(), "check_drawer_permission for load_keys[0]: %s", load_keys[0]); //DEBUGGING
      
    auto authentication_request = AuthenticateUser::Goal();
    authentication_request.permission_keys =  loading ? load_keys : drop_of_keys;
    
    auto send_goal_options = rclcpp_action::Client<AuthenticateUser>::SendGoalOptions();
    send_goal_options.goal_response_callback = bind(&DrawerManager::authentication_goal_response_callback, this, placeholders::_1);
    send_goal_options.feedback_callback = bind(&DrawerManager::authentication_feedback_callback, this, placeholders::_1, placeholders:: _2);
   
    return this->authenticate_user_client->async_send_goal(authentication_request, send_goal_options);
  }


  string DrawerManager::wait_for_user_authentication(AuthenticateUserResultHandle action_handle)
  {
    auto action_result = this->authenticate_user_client->async_get_result(action_handle.get());
    auto result = action_result.get();

    //TODO: What do we do in case of aborted or canceled?
    switch (result.code) 
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "AuthenticateUser Goal was aborted");
        return"";
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "AuthenticateUser Goal was canceled");
        return"";
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code for AuthenticateUser");
        return"";
    }

    RCLCPP_INFO(this->get_logger(), "AuthenticateUser action done "); //DEBUGGING
    return action_result.get().result.get()->permission_key_used;
  }
 

  void DrawerManager::authentication_goal_response_callback (const GoalHandleAuthenticateUser::SharedPtr & goal_handle)
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

  
  void DrawerManager::authentication_feedback_callback (GoalHandleAuthenticateUser::SharedPtr, const std::shared_ptr<const AuthenticateUser::Feedback> feedback)
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


  DrawerManager::DrawerUserAccessResultHandle DrawerManager::request_drawer_user_access(DrawerAddress drawer_address)
  {
    auto drawer_user_access_request = DrawerUserAccess::Goal();
    drawer_user_access_request.drawer_address = drawer_address;
    drawer_user_access_request.state = 1; //DEBUGGING
    
    auto send_goal_options = rclcpp_action::Client<DrawerUserAccess>::SendGoalOptions();
    send_goal_options.goal_response_callback = bind(&DrawerManager::open_drawer_goal_response_callback, this, placeholders::_1);
    send_goal_options.feedback_callback = bind(&DrawerManager::open_drawer_feedback_callback, this, placeholders::_1, placeholders:: _2);
    RCLCPP_INFO(this->get_logger(), "Start drawer user access with drawer controller");
    return this->user_drawer_access_client->async_send_goal(drawer_user_access_request, send_goal_options);
  }


  void DrawerManager::wait_for_finished_drawer_user_access(DrawerUserAccessResultHandle drawer_user_access_action_handle)
  {
    auto action_result = this->user_drawer_access_client->async_get_result(drawer_user_access_action_handle.get());
    auto result = action_result.get();

    //TODO: What do we do in case of aborted or canceled?
    switch (result.code) 
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "DrawerUserAccess Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "DrawerUserAccess Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code for DrawerUserAccess");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "DrawerUserAccess action done "); //DEBUGGING
    return;
  }


  void DrawerManager::ask_user_for_reopening_drawer(const std::shared_ptr<const robast_ros2_msgs::action::DrawerInteraction_Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "UserInterface:: Wollen sie die Schublade noch mal öffnen?");
    //TODO: Implement this
    if (false) 
    {
      uint8_t state = 2;
      this->drawer_interaction_state_machine(goal, state);
    }
    else
    {
      return;
    }    
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


  void DrawerManager::remind_user_to_close_drawer()
  {
    RCLCPP_INFO(this->get_logger(), "UserInterface::Bitte schließen sie die Schublade nachdem sie alles entnommen haben.");
  }

} // namespace robast