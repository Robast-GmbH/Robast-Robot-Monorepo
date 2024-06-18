#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <create_pose_from_spatial_detections/create_pose_from_spatial_detections.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace create_pose_from_spatial_detections
{
class CreatePoseFromSpatialDetectionsBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<CreatePoseFromSpatialDetections>(factory, "CreatePoseFromSpatialDetections", shared_resources);
    
  }
};
}  // namespace create_pose_from_spatial_detections

PLUGINLIB_EXPORT_CLASS(create_pose_from_spatial_detections::CreatePoseFromSpatialDetectionsBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
