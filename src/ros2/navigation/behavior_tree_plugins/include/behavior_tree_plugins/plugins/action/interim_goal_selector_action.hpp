#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INTERIM_GOAL_SELECTOR_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INTERIM_GOAL_SELECTOR_ACTION_HPP_

#include <string>

#include "communication_interfaces/action/compute_interim_goal.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{
  class InterimGoalCompAction : public BtActionNode<communication_interfaces::action::ComputeInterimGoal>
  {
   public:
    InterimGoalCompAction(const std::string& xml_tag_name,
                          const std::string& action_name,
                          const BT::NodeConfiguration& conf);

    void on_tick() override;

    void on_wait_for_result();

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("interim_goal",
                                                        "Interim goal selected by the interim_goal_selector node"),
        BT::OutputPort<int>("waypoint_index",
                            "Waypoint index of the path-waypoints that was closest to the chosen interim pose"),
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to Goal"),
        BT::InputPort<bool>("is_path_reversed", "Is the supplied path reversed?"),
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("interim_poses", "Interim poses list"),
        BT::InputPort<double>("search_radius", "Radius to search for interim_poses"),
      });
    }
  };

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INTERIM_GOAL_SELECTOR_ACTION_HPP_