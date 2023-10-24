#ifndef ROBOT_CLIENT__NFC_TASK_HPP_
#define ROBOT_CLIENT__NFC_TASK_HPP_

#include <memory>
#include <string>
#include <vector>

#include "base_task.hpp"
#include "communication_interfaces/action/create_user_nfc_tag.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace rmf_robot_client
{
  class NFCTask : public BaseTask
  {
   public:
    using CreateUserNfcTag = communication_interfaces::action::CreateUserNfcTag;
    using GoalHandleCreateUserNfcTag = rclcpp_action::ClientGoalHandle<CreateUserNfcTag>;

    NFCTask(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, int user_id);

    bool start(std::function<void(int)> next_task_callback) override;
    bool cancel();
    std::string get_type();
    bool receive_new_settings(std::string command, std::vector<std::string> value) override;
    void feedback_callback(GoalHandleCreateUserNfcTag::SharedPtr,
                           const std::shared_ptr<const CreateUserNfcTag::Feedback> feedback);
    void result_callback(const GoalHandleCreateUserNfcTag::WrappedResult& result);

    void start_writing_procedure();
    void goal_response_callback(const GoalHandleCreateUserNfcTag::SharedPtr& goal_handle);

   private:
    int user_id;

    rclcpp_action::Client<CreateUserNfcTag>::SharedPtr _nfc_write_new_nfc_card_client;
    GoalHandleCreateUserNfcTag::SharedPtr _current_action_goal_handle;
  };
}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__NFC_TASK_HPP_
