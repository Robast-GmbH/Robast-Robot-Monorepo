// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <setup_mtc_move_to_pose_with_constraints/setup_mtc_move_to_pose_with_constraints.hpp>

#include <string>
#include <memory>

#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>

#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace
{
inline constexpr auto kDescriptionSetupMtcMoveToPoseWithConstraints = R"(
                <p>
                    Given an existing MTC Task object and a target pose, appends MTC stages to describe a freespace motion plan to that target pose.
                </p>
            )";
constexpr auto kPortIdTask = "task";
constexpr auto kPortIdPlanningGroupName = "planning_group_name";
constexpr auto kPortIdIkFrame = "ik_frame";
constexpr auto kPortIdTargetPose = "target_pose";
constexpr auto kPortIDUseAllPlanners = "use_all_planners";
constexpr auto kPortIDConstraints = "constraints";

constexpr auto kMaxSolutions = 8;
constexpr auto kTimeOutSecondsIk = 1.0;

constexpr auto kPropertyNameTrajectoryExecutionInfo = "trajectory_execution_info";

}  // namespace

namespace moveit_studio::behaviors
{
SetupMtcMoveToPoseWithConstraints::SetupMtcMoveToPoseWithConstraints(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SetupMtcMoveToPoseWithConstraints::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIdIkFrame, "grasp_link",
                               "Name of the frame that is moved to align with the goal pose."),
    BT::InputPort<std::string>(kPortIdPlanningGroupName, "manipulator", "Name of the MoveIt planning group."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIdTargetPose, "{target_pose}", "Goal pose."),
    BT::InputPort<bool>(kPortIDUseAllPlanners, false, "Use all available planners in parallel to find a solution."),
    BT::InputPort<std::shared_ptr<moveit_msgs::msg::Constraints>>(
        kPortIDConstraints, std::make_shared<moveit_msgs::msg::Constraints>(), "Motion planning constraints."),
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortIdTask, "{mtc_task}", "MoveIt Task Constructor task.")
  };
}

BT::KeyValueVector SetupMtcMoveToPoseWithConstraints::metadata()
{
  return { { "subcategory", "Task Planning" }, { "description", kDescriptionSetupMtcMoveToPoseWithConstraints } };
}

BT::NodeStatus SetupMtcMoveToPoseWithConstraints::tick()
{
  const auto task = getInput<moveit::task_constructor::TaskPtr>(kPortIdTask);
  const auto planning_group_name = getInput<std::string>(kPortIdPlanningGroupName);
  const auto ik_frame = getInput<std::string>(kPortIdIkFrame);
  const auto target_pose = getInput<geometry_msgs::msg::PoseStamped>(kPortIdTargetPose);
  const auto use_all_planners = getInput<bool>(kPortIDUseAllPlanners);
  const auto constraints = getInput<std::shared_ptr<moveit_msgs::msg::Constraints>>(kPortIDConstraints);

  // Check that all required input data ports were set
  if (const auto error = maybe_error(task, planning_group_name, ik_frame, target_pose))
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 error.value());
    return BT::NodeStatus::FAILURE;
  }

  auto container = std::make_unique<moveit::task_constructor::SerialContainer>("Move to Pose");
  // TODO (3491): Fix exposing trajectory_execution_info property set in top-level Task into all containers
  // to avoid needing to explicitly set it like this.
  container->properties().set(kPropertyNameTrajectoryExecutionInfo,
                              boost::any_cast<moveit::task_constructor::TrajectoryExecutionInfo>(
                                  task.value()->properties().get(kPropertyNameTrajectoryExecutionInfo)));
  container->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
  container->setProperty("group", planning_group_name.value());

  // Create planners
  const auto mtc_pipeline_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(
      shared_resources_->node);

  /** Move To Pose **/
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::Connect>(
        "move to pose", moveit::task_constructor::stages::Connect::GroupPlannerVector{
                            { planning_group_name.value(), mtc_pipeline_planner } });
    if (constraints)
    {
      stage->setPathConstraints(*(constraints.value()));
    }
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    container->add(std::move(stage));
  }

  /** Generate Joint State at Pose  **/
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate pose");
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    stage->setPose(target_pose.value());
    stage->setMonitoredStage(task.value()->stages()->findChild("current state"));  // Hook into current state

    // Compute IK
    auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(kMaxSolutions);
    wrapper->setTimeout(kTimeOutSecondsIk);
    wrapper->setIKFrame(ik_frame.value());
    wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
    container->add(std::move(wrapper));
  }

  task.value()->add(std::move(container));

  return BT::NodeStatus::SUCCESS;
}
}  // namespace moveit_studio::behaviors