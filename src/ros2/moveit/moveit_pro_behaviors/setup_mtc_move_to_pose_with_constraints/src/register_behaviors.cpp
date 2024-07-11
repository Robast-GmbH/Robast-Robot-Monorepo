#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <setup_mtc_move_to_pose_with_constraints/setup_mtc_move_to_pose_with_constraints.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace moveit_studio::behaviors
{
class SetupMtcMoveToPoseWithConstraintsBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<SetupMtcMoveToPoseWithConstraints>(factory, "SetupMtcMoveToPoseWithConstraints", shared_resources);
    
  }
};
}  // namespace moveit_studio::behaviors

PLUGINLIB_EXPORT_CLASS(moveit_studio::behaviors::SetupMtcMoveToPoseWithConstraintsBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
