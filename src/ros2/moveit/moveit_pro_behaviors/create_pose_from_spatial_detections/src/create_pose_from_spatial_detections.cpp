#include <create_pose_from_spatial_detections/create_pose_from_spatial_detections.hpp>

namespace create_pose_from_spatial_detections
{
constexpr auto PORT_ID_SPATIAL_DETECTIONS = "spatial_detections";
constexpr auto PORT_ID_POSE_STAMPED_OUT = "pose_stamped_out";
constexpr auto PORT_ID_CONFIDENCE_THRESHOLD = "confidence_threshold";

CreatePoseFromSpatialDetections::CreatePoseFromSpatialDetections(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}


BT::PortsList CreatePoseFromSpatialDetections::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<double>(PORT_ID_CONFIDENCE_THRESHOLD, 0.5, "The confidence threshold to consider a detection."),
      BT::InputPort<depthai_ros_msgs::msg::SpatialDetectionArray>(PORT_ID_SPATIAL_DETECTIONS, "{spatial_detections}", "The spatial detections to create a pose from."),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(PORT_ID_POSE_STAMPED_OUT, "{pose_stamped}", "The output pose stamped."),
  });
}

BT::KeyValueVector CreatePoseFromSpatialDetections::metadata()
{
  return { {"description", "Creates a pose from given spatial detections. The detection with the highest confidence is taken to create the pose."} };
}

BT::NodeStatus CreatePoseFromSpatialDetections::onStart()
{
  shared_resources_->logger->publishInfoMessage(name(), "CreatePoseFromSpatialDetections::onStart!", false); //TODO: REMOVE THIS LINE

  const BT::Expected<depthai_ros_msgs::msg::SpatialDetectionArray> spatial_detections = getInput<depthai_ros_msgs::msg::SpatialDetectionArray>(PORT_ID_SPATIAL_DETECTIONS);
  const BT::Expected<double> confidence_threshold = getInput<double>(PORT_ID_CONFIDENCE_THRESHOLD);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(confidence_threshold, spatial_detections); error)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 error.value());
    return BT::NodeStatus::FAILURE;
  }

  // Check that the confidence threshold is valid
  if (confidence_threshold.value() < 0 || confidence_threshold.value() > 1)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Confidence threshold must be between 0 and 1.");
    return BT::NodeStatus::FAILURE;
  }

  // Check that the spatial detections are not empty
  if (spatial_detections.value().detections.empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "No spatial detections found.");
    return BT::NodeStatus::FAILURE;
  }

  // Find the detection with the highest score and above the threshold
  depthai_ros_msgs::msg::SpatialDetection highest_confidence_detection = *std::max_element(
      spatial_detections.value().detections.begin(), spatial_detections.value().detections.end(),
      [](const depthai_ros_msgs::msg::SpatialDetection& a, const depthai_ros_msgs::msg::SpatialDetection& b) {
        return a.results[0].score > b.results[0].score;
      });

  // Check if the highest confidence detection is above the threshold
  if (highest_confidence_detection.results[0].score < confidence_threshold.value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "No detections above the confidence threshold.");
    return BT::NodeStatus::FAILURE;
  }

  // Create the pose from the highest confidence detection
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = spatial_detections.value().header;
  pose_stamped.pose.position = highest_confidence_detection.position;

  // Publish the pose
  setOutput<geometry_msgs::msg::PoseStamped>(PORT_ID_POSE_STAMPED_OUT, pose_stamped);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CreatePoseFromSpatialDetections::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void CreatePoseFromSpatialDetections::onHalted()
{
}

}  // namespace create_pose_from_spatial_detections
