// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>
#include <memory>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

namespace moveit_studio::behaviors
{
/**
 * @brief Given an existing MTC Task object and a target pose, appends MTC stages to describe a freespace motion
 * plan to that target pose.
 *
 * @details
 * | Data Port Name      | Port Type     | Object Type                                     |
 * | ------------------- |---------------|-------------------------------------------------|
 * | task                | Bidirectional | std::shared_ptr<moveit::task_constructor::Task> |
 * | ik_frame            | Input         | std::string                                     |
 * | planning_group_name | Input         | std::string                                     |
 * | target_pose         | Input         | geometry_msgs::msg::PoseStamped                 |
 * | use_all_planners    | Input         | bool                                            |
 * | constraints         | Input         | std::shared_ptr<moveit_msgs::msgs::Constraints> |
 */
class SetupMtcMoveToPoseWithConstraints final : public SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Constructor for GetLatestTransform behavior.
   * @param name Name of the node. Must match the name used for this node
   * in the behavior tree definition file (the .xml file).
   * @param config Node configuration. Only used here because the BehaviorTree.CPP
   * expects constructor signature with name and config first before custom
   * constructor parameters.
   * @param shared_resources Provides access to common resources such as the node handle
   * and failure logger that are shared between all the behaviors that inherit from
   * moveit_studio::behaviors::SharedResourcesNode.
   */
  SetupMtcMoveToPoseWithConstraints(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<BehaviorContext>& shared_resources);

  /**
   * @brief Custom tree nodes that have input and/or output ports must define
   * them in this static function.
   * @details This function must be static. It is a requirement set by the
   * BehaviorTree.CPP library.
   * @return List of ports with the names and port info. The return value is
   * for the internal use of the behavior tree.
   */
  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  /**
   * @brief Synchronous, blocking function that must return SUCCESS/FAILURE
   * every time this action node is ticked by the behavior tree.
   * @return Status of the node. Must be either SUCCESS/FAILURE (the node is
   * "complete" for now). Cannot be RUNNING since this is a synchronous node.
   */
  BT::NodeStatus tick() override;
};
}  // namespace moveit_studio::behaviors