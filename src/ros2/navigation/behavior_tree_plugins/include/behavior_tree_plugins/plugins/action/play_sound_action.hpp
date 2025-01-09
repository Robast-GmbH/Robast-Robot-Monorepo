#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PLAY_SOUND_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PLAY_SOUND_ACTION_HPP_

#include <string>

#include "communication_interfaces/action/play_sound.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

  class PlaySoundAction : public BtActionNode<communication_interfaces::action::PlaySound>
  {
   public:
    PlaySoundAction(const std::string& xml_tag_name, const std::string& action_name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({BT::InputPort<std::string>("file_name", "Path to the sound file to play")});
    }
  };

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PLAY_SOUND_ACTION_HPP_