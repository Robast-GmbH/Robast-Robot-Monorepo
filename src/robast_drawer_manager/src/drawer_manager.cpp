#include <memory>

#include "robast_drawer_manager/drawer_manager.hpp"

namespace robast_drawer_manager
{
  

  DrawerManager::DrawerManager(): Node("drawer_manager_Node")
  {
    RCLCPP_INFO(get_logger(), "Creating");
    this->get_all_drawers_server = this->create_service<robast_msgs::srv::GetAllAttachedDrawers>("Get_all_attached_drawer_drawer_management", &get_all_drawer);
    this->handle_drawer_server = this->create_service<robast_msgs::srv::HandleDrawer>("open_drawer_drawer_management", &open_drawer);
    
    this->authenticate_user_client = this->create_client<robast_msgs::srv::AuthenticateUser>("authenticate_drawer_user");
    this->open_drawers_client = rclcpp_action::create_client<robast_msgs::srv::DrawerUserAccess>(this,"open_drawer_drawer");
  }

  DrawerManager::~DrawerManager()
  {
    RCLCPP_INFO(get_logger(), "Destroying");
  }

  void get_all_drawer(const std::shared_ptr<robast_msgs::srv::GetAllAttachedDrawers::Request> request, std::shared_ptr<robast_msgs::srv::GetAllAttachedDrawers::Response>      response)
  {
    rclcpp::Client<robast_msgs::srv::GetAllAttachedDrawers>::SharedPtr stateMaschineRequest =  node->create_client<robast_msgs::srv::GetAllAttachedDrawers>("Get_all_attached_drawer");
    response->drawers;
  }

  void open_drawer(const std::shared_ptr<robast_msgs::srv::HandleDrawer::Request> request,
                      std::shared_ptr<robast_msgs::srv::HandleDrawer::Response>      response)
  {
    // NFC Reader
    auto authentication_request = std::make_shared<robast_msgs::srv::AuthenticateUser::Request>();
    authentication_request.permission_keys = request.permission_keys;

    auto result =  this->authenticate_user_client ->async_send_request(authentication_request);
    RCLCPP_INFO(get_logger(), "The reader is active please authenticate yourselve at the reader.");
    if (!rclcpp::spin_until_future_complete(this, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to reach the NFC module");
      return 
    }

    if(!result.get()->sucessful)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), result.get()->error_message);
      return 
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
    send_goal_options.feedback_callback = std::bind(&DrawerManager::open_drawer_feedback_callback, this, _1, _2);
    auto result =this->open_drawers_client->async_send_goal(target_drawer_msg, send_goal_options);

    // Schublade offen im Frontend anzeigen
    RCLCPP_INFO(get_logger(), "The drawer is open please close the drawer after you are done.");
      
    // nach zeit intervall //bitte schließen imfrontend anzeigen
    this->door_timer = this->create_wall_timer(  std::chrono::milliseconds(500), std::bind(&DrawerManager::remind_user_to_close_drawer, this));
    
    if (!rclcpp::spin_until_future_complete(this, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to reach the drawer gate");
      return 
    }
    
    //  wenn die Schublade  geschlossen ist action beenden 
  }



  void open_drawer_feedback_callback( rclcpp_action::ClientGoalHandle<robast_msgs::action::DrawerUserAccess>::SharedPtr, const std::shared_ptr<const robast_msgs::action::DrawerUserAccess::Feedback> feedback);
  {
      if(feedback->is_drawer_open())
      { 
        this->door_timer = this->create_wall_timer(  std::chrono::milliseconds(500), std::bind(&DrawerManager::remind_user_to_close_drawer, this));
      }
      else 
      {
        this->door_timer->cancel();
      }
  }

  

  void remind_user_to_close_drawer()
  {
    RCLCPP_INFO(get_logger(), "Please close the drawer when you are done.");
  }

} // namespace robast_drawer_manager