#include "audio_bridge/sound_playback.hpp"

#include <cstdlib>
#include <thread>

SoundPlayerNode::SoundPlayerNode() : Node("sound_player_node")
{
  this->_action_server = rclcpp_action::create_server<PlaySound>(
      this,
      "play_sound",
      std::bind(&SoundPlayerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SoundPlayerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&SoundPlayerNode::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse SoundPlayerNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                         std::shared_ptr<const PlaySound::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with sound file: %s", goal->sound_file.c_str());
  (void) uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SoundPlayerNode::handle_cancel(const std::shared_ptr<GoalHandlePlaySound> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void) goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SoundPlayerNode::handle_accepted(const std::shared_ptr<GoalHandlePlaySound> goal_handle)
{
  using namespace std::placeholders;
  std::jthread{std::bind(&SoundPlayerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void SoundPlayerNode::execute(const std::shared_ptr<GoalHandlePlaySound> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PlaySound::Feedback>();
  auto result = std::make_shared<PlaySound::Result>();

  // Play the sound file using mpg123 with PulseAudio
  std::string command = "mpg123 -o pulse sounds/" + goal->sound_file;
  int ret = std::system(command.c_str());

  if (ret != 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to play sound file");
    result->success = false;
    result->message = "Failed to play sound file";
    goal_handle->abort(result);
    return;
  }

  feedback->feedback = "Playing sound file: " + goal->sound_file;
  goal_handle->publish_feedback(feedback);

  result->success = true;
  result->message = "Sound file played successfully";
  goal_handle->succeed(result);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SoundPlayerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}