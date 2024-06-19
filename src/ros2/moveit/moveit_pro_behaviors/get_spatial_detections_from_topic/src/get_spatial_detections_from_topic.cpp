#include <get_spatial_detections_from_topic/get_spatial_detections_from_topic.hpp>

// Include the template implementation for GetMessageFromTopicBehaviorBase<T>.
#include <moveit_studio_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace get_spatial_detections_from_topic
{
constexpr auto PORT_ID_TIMEOUT_DURATION = "timeout_duration";
constexpr auto PORT_ID_MESSAGE_OUT = "message_out"; // THIS MAY NOT BE CHANGED AS IT IS USED IN THE TEMPLATE IMPLEMENTATION

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
      BT::InputPort<double>(PORT_ID_TIMEOUT_DURATION, 5.0, "The duration to wait for a message before failing."),
      BT::OutputPort<depthai_ros_msgs::msg::SpatialDetectionArray>(PORT_ID_MESSAGE_OUT, "{spatial_detections}", "The output spatial detections."),
  });
}

BT::KeyValueVector GetSpatialDetectionsFromTopic::metadata()
{
  return { {"description", "Listens to a given topic for spatial detections and makes it available on an output port."} };
}

tl::expected<std::chrono::duration<double>, std::string> GetSpatialDetectionsFromTopic::getWaitForMessageTimeout()
{
  return std::chrono::duration<double>{
    getInput<double>(PORT_ID_TIMEOUT_DURATION).value_or(5.0)
  };
}

}  // namespace get_spatial_detections_from_topic

template class moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<depthai_ros_msgs::msg::SpatialDetectionArray>;
