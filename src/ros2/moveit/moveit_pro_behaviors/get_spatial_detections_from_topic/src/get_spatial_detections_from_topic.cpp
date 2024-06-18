#include <get_spatial_detections_from_topic/get_spatial_detections_from_topic.hpp>

// Include the template implementation for GetMessageFromTopicBehaviorBase<T>.
#include <moveit_studio_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace get_spatial_detections_from_topic
{
GetSpatialDetectionsFromTopic::GetSpatialDetectionsFromTopic(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) :
   GetMessageFromTopicBehaviorBase<depthai_ros_msgs::msg::SpatialDetectionArray>(name, config, shared_resources)
{
}


BT::PortsList GetSpatialDetectionsFromTopic::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<std::string>("topic_name", "/stereo/door_handle_position", "The name of the topic the spatial detections are published on."),
      BT::OutputPort<depthai_ros_msgs::msg::SpatialDetectionArray>("spatial_detections_out", "{spatial_detections}", "The output spatial detections."),
  });
}

BT::KeyValueVector GetSpatialDetectionsFromTopic::metadata()
{
  return { {"description", "Listens to a given topic for spatial detections and makes it available on an output port."} };
}

}  // namespace get_spatial_detections_from_topic

template class moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<depthai_ros_msgs::msg::SpatialDetectionArray>;
