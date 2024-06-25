#include <create_pose_from_spatial_detections/create_pose_from_spatial_detections.hpp>
#include <tl_expected/expected.hpp>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
namespace create_pose_from_spatial_detections
{
constexpr auto PORT_ID_SPATIAL_DETECTIONS = "spatial_detections";
constexpr auto PORT_ID_POSE_STAMPED_OUT = "pose_stamped_out";
constexpr auto PORT_ID_CONFIDENCE_THRESHOLD = "confidence_threshold";


CreatePoseFromSpatialDetections::CreatePoseFromSpatialDetections(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
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

tl::expected<bool, std::string> CreatePoseFromSpatialDetections::doWork()
{

  const auto spatial_detections = getInput<depthai_ros_msgs::msg::SpatialDetectionArray>(PORT_ID_SPATIAL_DETECTIONS);
  const auto confidence_threshold = getInput<double>(PORT_ID_CONFIDENCE_THRESHOLD);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(confidence_threshold, spatial_detections); error)
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  // Check that the confidence threshold is valid
  if (confidence_threshold.value() < 0 || confidence_threshold.value() > 1)
  {
    return tl::make_unexpected("Confidence threshold must be between 0 and 1.");
  }

  // Check that the spatial detections are not empty
  if (spatial_detections.value().detections.empty())
  {
    return tl::make_unexpected("No spatial detections found.");
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
    return tl::make_unexpected("No detections above the confidence threshold.");
  }

  // Create the pose from the highest confidence detection
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = spatial_detections.value().header;
  pose_stamped.pose.position = highest_confidence_detection.position;

  setOutput(PORT_ID_POSE_STAMPED_OUT, pose_stamped);

  return true;
}




}  // namespace create_pose_from_spatial_detections