#include "robot_client/nfc_task.hpp"

namespace rmf_robot_client
{

  NFCTask::NFCTask(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, int user_id)
      : BaseTask(task_id, step, ros_node)
  {
    this->_user_id = user_id;
    _nfc_write_new_nfc_card_client = rclcpp_action::create_client<CreateUserNfcTag>(
        ros_node, ros_node_->get_parameter("nfc_write_usertag_to_card_action_topic").as_string());
  }

  bool NFCTask::start(std::function<void(int)> next_task_callback)
  {
    BaseTask::start(next_task_callback);
    RCLCPP_INFO(ros_node_->get_logger(), "start nfc task");
    finish_task_ = next_task_callback;
    if (!this->_nfc_write_new_nfc_card_client->wait_for_action_server())
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Action server not available after waiting");
      //  cancel task
      return false;
    }
    start_writing_procedure();
    return true;
  }

  void NFCTask::start_writing_procedure()
  {
    CreateUserNfcTag::Goal msg = CreateUserNfcTag::Goal();
    msg.user_id = _user_id;
    rclcpp_action::Client<CreateUserNfcTag>::SendGoalOptions send_goal_options =
        rclcpp_action::Client<CreateUserNfcTag>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(&NFCTask::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NFCTask::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&NFCTask::result_callback, this, std::placeholders::_1);

    this->_nfc_write_new_nfc_card_client->async_send_goal(msg, send_goal_options);
  }

  void NFCTask::goal_response_callback(const GoalHandleCreateUserNfcTag::SharedPtr& goal_handle)
  {
    _current_action_goal_handle = goal_handle;
    if (!_current_action_goal_handle)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server");
      publish_task_state("Canceld", "could not plan route to goal pose", true);
      finish_task_(false);
    }
    else
    {
      RCLCPP_INFO(ros_node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void NFCTask::feedback_callback(GoalHandleCreateUserNfcTag::SharedPtr,
                                  const std::shared_ptr<const CreateUserNfcTag::Feedback>)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "new user feedback recived");
  }

  void NFCTask::result_callback(const GoalHandleCreateUserNfcTag::WrappedResult& result)
  {
    if (result.result->successful)
    {
      RCLCPP_INFO(ros_node_->get_logger(), "new user done");
      publish_task_state("Completed", "card_created", true);
      finish_task_(true);
    }
    else
    {
      publish_task_state("Canceld", "", true);
      finish_task_(false);
    }
  }

  bool NFCTask::cancel()
  {
    this->_nfc_write_new_nfc_card_client->async_cancel_goal(_current_action_goal_handle);
    publish_task_state("Canceld", "", true);
    finish_task_(false);
    return true;
  }

  bool NFCTask::receive_new_settings(std::string command, std::vector<std::string> value)
  {
    if (BaseTask::receive_new_settings(command, value))
    {
      return true;
    }
    return false;
  }

  std::string NFCTask::get_type()
  {
    return "NFC_TASK";
  }
}   // namespace rmf_robot_client
