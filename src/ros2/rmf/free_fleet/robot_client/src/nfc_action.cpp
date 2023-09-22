#include "robot_client/nfc_action.hpp"

namespace rmf_robot_client
{

  NFCAction::NFCAction(int task_id, int step,std::shared_ptr<rclcpp::Node>  ros_node, std::map<std::string, std::string> config, int user_id) : Action(task_id, step, ros_node, config)
  {
    this->user_id = user_id;
     nfc_write_new_nfc_card_client_ = rclcpp_action::create_client<CreateUserNfcTag>(ros_node, config["nfc_write_usertag_to_card_action_topic"]);
  }

  bool NFCAction::start(std::function<void(int)> next_action_callback)
  {
     RCLCPP_INFO(ros_node_->get_logger(), "start drawer_action");
    finish_action = next_action_callback;
    if (!this->nfc_write_new_nfc_card_client_->wait_for_action_server()) 
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Action server not available after waiting");
      //cancel task
      return false;
    }
    start_writing_procedure();
    return true;
  }

  void  NFCAction::start_writing_procedure()
  {
    CreateUserNfcTag::Goal msg = CreateUserNfcTag::Goal();
    msg.first_name = "";
    msg.last_name = "";
    msg.user_id = user_id;
    rclcpp_action::Client<CreateUserNfcTag>::SendGoalOptions send_goal_options = rclcpp_action::Client<CreateUserNfcTag>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(&NFCAction::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&NFCAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&NFCAction::result_callback, this, std::placeholders::_1);
 
    this->nfc_write_new_nfc_card_client_->async_send_goal(msg, send_goal_options);
  }

   void NFCAction::goal_response_callback(const GoalHandleCreateUserNfcTag::SharedPtr &goal_handle)
  {
    current_action_goal_handle = goal_handle;
    if (!current_action_goal_handle) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server");
      publish_task_state("Canceld", "could not plan route to goal pose", true);
      finish_action(false);
    }
    else
    {
      RCLCPP_INFO(ros_node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void NFCAction::feedback_callback(GoalHandleCreateUserNfcTag::SharedPtr, const std::shared_ptr<const CreateUserNfcTag::Feedback> feedback)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "navigate_to_pose feedback recived");
  }

  void NFCAction::result_callback(const GoalHandleCreateUserNfcTag::WrappedResult &result)
  {
    if(result.result->successful)
    {
      publish_task_state("Completed", "card_created", true);
      finish_action(true);
    }
    else
    {
      publish_task_state("Canceld", "", true);
      finish_action(false);
    }
  }

  
  bool NFCAction::cancel()
  {
    this->nfc_write_new_nfc_card_client_->async_cancel_goal(current_action_goal_handle);
    publish_task_state("Canceld", "", true);
    finish_action(false);
    return true;
  }
  
  bool NFCAction::receive_new_settings(std::string command, std::vector<std::string> value)
  {
    if(Action::receive_new_settings(command,value))
    {
      return true;
    }
  }
  
  std::string NFCAction::get_type()
  {
    return "NFC_ACTION";
  }
}