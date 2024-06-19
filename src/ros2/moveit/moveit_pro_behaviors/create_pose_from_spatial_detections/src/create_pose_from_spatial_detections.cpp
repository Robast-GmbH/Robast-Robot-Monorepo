#include <create_pose_from_spatial_detections/create_pose_from_spatial_detections.hpp>
#include <tl_expected/expected.hpp>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
namespace create_pose_from_spatial_detections
{
constexpr auto PORT_ID_SPATIAL_DETECTIONS = "spatial_detections";
constexpr auto PORT_ID_POSE_STAMPED_OUT = "pose_stamped_out";
constexpr auto PORT_ID_CONFIDENCE_THRESHOLD = "confidence_threshold";
constexpr auto PORT_ID_PUBLISH_POSE = "publish_pose";
constexpr auto PORT_ID_TARGET_FRAME_FOR_POSE_STAMPED = "target_frame_pose_stamped";

const std::string TOPIC_POSE_STAMPED = "/debug/pose_stamped";

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
      BT::InputPort<bool>(PORT_ID_PUBLISH_POSE, false, "Whether to publish the pose for debugging purposes to the topic " + TOPIC_POSE_STAMPED + "."),
      BT::InputPort<std::string>(PORT_ID_TARGET_FRAME_FOR_POSE_STAMPED, "base_footprint", "The target frame for the pose stamped. Only applied to the pose published for debugging purposes."),
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
  const auto publish_pose = getInput<bool>(PORT_ID_PUBLISH_POSE);
  const auto taget_frame_pose_stamped = getInput<std::string>(PORT_ID_TARGET_FRAME_FOR_POSE_STAMPED);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(confidence_threshold, spatial_detections, publish_pose, taget_frame_pose_stamped); error)
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

  if (publish_pose.value())
  {
    // check if the publisher was already created
    if (!_pose_stamped_publisher)
    {    
      _pose_stamped_publisher = shared_resources_->node->create_publisher<geometry_msgs::msg::PoseStamped>(TOPIC_POSE_STAMPED, rclcpp::QoS(1));
      _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->shared_resources_->node->get_clock());
      _transform_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    }

    // Transform the pose to the target frame
    geometry_msgs::msg::PoseStamped pose_stamped_transformed = transform_pose(pose_stamped, taget_frame_pose_stamped.value());

    _pose_stamped_publisher->publish(pose_stamped_transformed);
  }

  setOutput(PORT_ID_POSE_STAMPED_OUT, pose_stamped);

  return true;
}

 geometry_msgs::msg::PoseStamped CreatePoseFromSpatialDetections::transform_pose(const geometry_msgs::msg::PoseStamped &pose_stamped, const std::string &target_frame)
 {
    geometry_msgs::msg::PoseStamped transformed_pose;
    int max_retry = 5;
    int retry_count = 0;
    while (retry_count < max_retry)
    {
      try
      {
        geometry_msgs::msg::TransformStamped transform_stamped = _tf_buffer->lookupTransform(target_frame, pose_stamped.header.frame_id, rclcpp::Time(0));
        
        tf2::doTransform(pose_stamped, transformed_pose, transform_stamped);
        break; // Break out of the loop if transformation is successful
      }
        catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(shared_resources_->node->get_logger(), "Transform error: %s", ex.what());
        rclcpp::sleep_for(std::chrono::milliseconds(200)); // most of the time the transform is not available yet, so wait a bit
        retry_count++;
      }
    }
    if (retry_count == max_retry)
    {
      RCLCPP_ERROR(shared_resources_->node->get_logger(), "Failed to transform pose after %d retries", max_retry);
    }
    return transformed_pose;
 }


}  // namespace create_pose_from_spatial_detections