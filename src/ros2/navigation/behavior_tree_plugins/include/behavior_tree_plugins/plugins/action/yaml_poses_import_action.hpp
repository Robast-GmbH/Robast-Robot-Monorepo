#ifndef POSES_IMPORT_ACTION_HPP_
#define POSES_IMPORT_ACTION_HPP_

#include <string>

#include "communication_interfaces/action/import_yaml_poses.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

  class ImportYamlPosesAction : public BtActionNode<communication_interfaces::action::ImportYamlPoses>
  {
   public:
    ImportYamlPosesAction(const std::string& xml_tag_name,
                          const std::string& action_name,
                          const BT::NodeConfiguration& conf);

    void on_tick() override;

    // void on_wait_for_result() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({
        BT::InputPort<std::string>("yaml_name", "name of the yaml containing poses"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses", "poses loaded of the yaml"),
      });
    }
  };

}   // namespace nav2_behavior_tree

#endif   // POSES_IMPORT_ACTION_HPP_