#ifndef RMF__ROBOT_CLIENT__NFC_ACTION_HPP_
#define RMF__ROBOT_CLIENT__NFC_ACTION_HPP_

#include "action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "communication_interfaces/action/create_user_nfc_tag.hpp"

namespace rmf_robot_client
{
  class NFCAction : public Action
  {
    private:
      int user_id;
      using CreateUserNfcTag = communication_interfaces::action::CreateUserNfcTag;
      using GoalHandleCreateUserNfcTag = rclcpp_action::ClientGoalHandle<CreateUserNfcTag>;
      rclcpp_action::Client<CreateUserNfcTag>::SharedPtr nfc_write_new_nfc_card_client_;
      GoalHandleCreateUserNfcTag::SharedPtr current_action_goal_handle;

    public:
      NFCAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string, std::string> config, int user_id);
      //~NFCAction();

      bool start(std::function<void(int)> next_action_callback);
      bool cancel();
      std::string get_type();
      bool receive_new_settings(std::string command, std::vector<std::string> value);
      void feedback_callback(GoalHandleCreateUserNfcTag::SharedPtr, const std::shared_ptr<const CreateUserNfcTag::Feedback> feedback);
      void result_callback(const GoalHandleCreateUserNfcTag::WrappedResult &result);

      void start_writing_procedure();
      void goal_response_callback(std::shared_future<GoalHandleCreateUserNfcTag::SharedPtr> future);
  };
}// namespace robot_client
#endif   // RMF__ROBOT_CLIENT__NFC_ACTION_HPP_