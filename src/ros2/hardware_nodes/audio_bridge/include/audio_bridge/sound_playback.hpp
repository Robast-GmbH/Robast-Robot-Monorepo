#ifndef AUDIO_BRIDGE__SOUND_PLAYBACK_HPP_
#define AUDIO_BRIDGE__SOUND_PLAYBACK_HPP_

#include <communication_interfaces/action/play_sound.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

class SoundPlayerNode : public rclcpp::Node
{
 public:
  using PlaySound = communication_interfaces::action::PlaySound;
  using GoalHandlePlaySound = rclcpp_action::ServerGoalHandle<PlaySound>;

  SoundPlayerNode();

 private:
  rclcpp_action::Server<PlaySound>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const PlaySound::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePlaySound> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandlePlaySound> goal_handle);

  void execute(const std::shared_ptr<GoalHandlePlaySound> goal_handle);
};

#endif   // AUDIO_BRIDGE__SOUND_PLAYBACK_HPP_