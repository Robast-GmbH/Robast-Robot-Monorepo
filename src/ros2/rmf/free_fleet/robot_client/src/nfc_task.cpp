#include "robot_client/nfc_task.hpp"

namespace rmf_robot_client
{

  NFCTask::NFCTask(TaskId task_id, std::shared_ptr<rclcpp::Node> ros_node, int user_id_for_token)
      : BaseTask(task_id, ros_node)
  {
    this->_user_id_for_token = user_id_for_token;
    _nfc_write_new_nfc_token_for_client = rclcpp_action::create_client<CreateUserNfcTag>(
        ros_node, ros_node->get_parameter("nfc_write_usertag_to_card_action_topic").as_string());
  }

  void NFCTask::start()
  {
    RCLCPP_INFO(ros_node_->get_logger(), "start assign new user to token task");
    if (!this->_nfc_write_new_nfc_token_for_client->wait_for_action_server())
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Action server not available after waiting");
      //  cancel task
      return;
    }
    start_writing_procedure();
    return;
  }

  void NFCTask::start_writing_procedure()
  {
    CreateUserNfcTag::Goal msg = CreateUserNfcTag::Goal();
    msg.user_id = _user_id_for_token;
    rclcpp_action::Client<CreateUserNfcTag>::SendGoalOptions send_goal_options =
        rclcpp_action::Client<CreateUserNfcTag>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(&NFCTask::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NFCTask::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&NFCTask::result_callback, this, std::placeholders::_1);
    publish_task_state("Notification", "wait_for_nfc_token", false);
    this->_nfc_write_new_nfc_token_for_client->async_send_goal(msg, send_goal_options);
  }

  void NFCTask::goal_response_callback(const GoalHandleCreateUserNfcTag::SharedPtr& goal_handle)
  {
    _current_action_goal_handle = goal_handle;
    if (!_current_action_goal_handle)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server");
      publish_task_state("Canceld", "reader is not ready to create a token ", true);
      task_done(false);
    }
    else
    {
      RCLCPP_INFO(ros_node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void NFCTask::feedback_callback(GoalHandleCreateUserNfcTag::SharedPtr,
                                  const std::shared_ptr<const CreateUserNfcTag::Feedback>)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "new user feedback received");
  }

  void NFCTask::result_callback(const GoalHandleCreateUserNfcTag::WrappedResult& result)
  {
    if (result.result->successful)
    {
      RCLCPP_INFO(ros_node_->get_logger(), "new user done");
      publish_task_state("Notification", "nfc_token_received", false);
      publish_task_state("Completed", "token was created", true);
      start_next_phase();
      task_done(true);
    }
    else
    {
      publish_task_state("Canceld", "", true);
      task_done(false);
    }
  }

  bool NFCTask::cancel()
  {
    this->_nfc_write_new_nfc_token_for_client->async_cancel_goal(_current_action_goal_handle);
    publish_task_state("Canceld", "", true);
    task_done(false);
    return true;
  }

  bool NFCTask::receive_new_settings(std::string command, std::vector<std::string> value)
  {
    return BaseTask::receive_new_settings(command, value);
  }

  void NFCTask::task_done(bool is_completted)
  {
    if (is_completted)
    {
      start_next_phase();
    }
  }

  std::string NFCTask::get_type()
  {
    return "NFC_TASK";
  }
}   // namespace rmf_robot_client
