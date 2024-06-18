#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <get_spatial_detections_from_topic/get_spatial_detections_from_topic.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace get_spatial_detections_from_topic
{
class GetSpatialDetectionsFromTopicBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<GetSpatialDetectionsFromTopic>(factory, "GetSpatialDetectionsFromTopic", shared_resources);
    
  }
};
}  // namespace get_spatial_detections_from_topic

PLUGINLIB_EXPORT_CLASS(get_spatial_detections_from_topic::GetSpatialDetectionsFromTopicBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
