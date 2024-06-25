#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <publish_pose_to_topic/publish_pose_to_topic.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace publish_pose_to_topic
{
class PublishPoseToTopicBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<PublishPoseToTopic>(factory, "PublishPoseToTopic", shared_resources);
    
  }
};
}  // namespace publish_pose_to_topic

PLUGINLIB_EXPORT_CLASS(publish_pose_to_topic::PublishPoseToTopicBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
