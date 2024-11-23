#include "publish_pose_from_spatial_detection/publish_pose_from_spatial_detection.hpp"

namespace perception
{
  PublishPoseFromSpatialDetection::PublishPoseFromSpatialDetection() : Node("publish_pose_from_spatial_detection")
  {
    declare_parameters();
    get_parameters();
    setup_subscriptions();
    setup_publishers();

    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _transform_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
  }

  void PublishPoseFromSpatialDetection::declare_parameters()
  {
    declare_parameter<bool>("use_confidence_threshold", false);
    declare_parameter<double>("confidence_threshold", 0.5);
    declare_parameter<bool>("invert_y_axis", false);
    declare_parameter<std::vector<double>>("target_orientation_in_euler", {0.0, 0.0, 1.57079632679});
    declare_parameter<std::string>("target_frame_pose_stamped", "robot/base_footprint");
    declare_parameter<std::string>("topic_name_spatial_detections", "/stereo/door_handle_position");
    declare_parameter<std::string>("topic_name_pose_stamped", "/stereo/door_handle_pose");
  }

  void PublishPoseFromSpatialDetection::get_parameters()
  {
    _use_confidence_threshold = get_parameter("use_confidence_threshold").as_bool();
    _confidence_threshold = get_parameter("confidence_threshold").as_double();
    _invert_y_axis = get_parameter("invert_y_axis").as_bool();
    _target_orientation_in_euler = get_parameter("target_orientation_in_euler").as_double_array();
    _taget_frame_pose_stamped = get_parameter("target_frame_pose_stamped").as_string();
    _topic_name_spatial_detections = get_parameter("topic_name_spatial_detections").as_string();
    _topic_name_pose_stamped = get_parameter("topic_name_pose_stamped").as_string();
  }

  void PublishPoseFromSpatialDetection::setup_subscriptions()
  {
    _spatial_detections_sub = create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
      _topic_name_spatial_detections,
      10,
      std::bind(&PublishPoseFromSpatialDetection::callback_spatial_detections, this, std::placeholders::_1));
  }

  void PublishPoseFromSpatialDetection::setup_publishers()
  {
    _pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(_topic_name_pose_stamped, 10);
  }

  void PublishPoseFromSpatialDetection::callback_spatial_detections(
    const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg)
  {
    std::thread{std::bind(&PublishPoseFromSpatialDetection::handle_spatial_detection, this, std::placeholders::_1), msg}
      .detach();
  }

  void PublishPoseFromSpatialDetection::handle_spatial_detection(
    const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg)
  {
    // Check that the spatial detections are not empty
    if (msg->detections.empty())
    {
      RCLCPP_DEBUG(get_logger(), "No spatial detections received");
      return;
    }

    // Find the detection with the highest score and above the threshold
    depthai_ros_msgs::msg::SpatialDetection highest_confidence_detection = *std::max_element(
      msg->detections.begin(),
      msg->detections.end(),
      [](const depthai_ros_msgs::msg::SpatialDetection &a, const depthai_ros_msgs::msg::SpatialDetection &b)
      {
        return a.results[0].score > b.results[0].score;
      });

    // Check if the highest confidence detection is above the threshold
    if (_use_confidence_threshold && (highest_confidence_detection.results[0].score < _confidence_threshold))
    {
      RCLCPP_DEBUG(get_logger(), "No detections above the confidence threshold.");
      return;
    }

    // Check that the highest confidence detection has a valid position
    const double epsilon = 1e-6;
    if (std::abs(highest_confidence_detection.position.x) < epsilon &&
        std::abs(highest_confidence_detection.position.y) < epsilon &&
        std::abs(highest_confidence_detection.position.z) < epsilon)
    {
      RCLCPP_DEBUG(get_logger(), "No valid position in the highest confidence detection.");
      return;
    }

    // Create the pose from the highest confidence detection
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose.position.x = highest_confidence_detection.position.x;
    // Invert y-axis to match frame of reference if requested
    pose_stamped.pose.position.y =
      _invert_y_axis ? (-1) * highest_confidence_detection.position.y : highest_confidence_detection.position.y;
    pose_stamped.pose.position.z = highest_confidence_detection.position.z;

    // Set the orientation to the target orientation if provided
    if (_target_orientation_in_euler != std::vector<double>{0.0, 0.0, 0.0})
    {
      tf2::Quaternion q;
      q.setRPY(_target_orientation_in_euler[0], _target_orientation_in_euler[1], _target_orientation_in_euler[2]);
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();
    }

    // Transform the pose to the target frame
    std::optional<geometry_msgs::msg::PoseStamped> pose_stamped_transformed =
      transform_pose(pose_stamped, _taget_frame_pose_stamped);

    if (pose_stamped_transformed.has_value())
    {
      _pose_pub->publish(pose_stamped_transformed.value());
    }
  }

  std::optional<geometry_msgs::msg::PoseStamped> PublishPoseFromSpatialDetection::transform_pose(
    const geometry_msgs::msg::PoseStamped &pose_stamped, const std::string &target_frame)
  {
    geometry_msgs::msg::PoseStamped transformed_pose;
    const int max_retry = 5;
    int retry_count = 0;
    while (retry_count < max_retry)
    {
      try
      {
        const geometry_msgs::msg::TransformStamped transform_stamped =
          _tf_buffer->lookupTransform(target_frame, pose_stamped.header.frame_id, rclcpp::Time(0));

        tf2::doTransform(pose_stamped, transformed_pose, transform_stamped);
        break;   // Break out of the loop if transformation is successful
      }
      catch (tf2::TransformException &ex)
      {
        rclcpp::sleep_for(
          std::chrono::milliseconds(200));   // most of the time the transform is not available yet, so wait a bit
        retry_count++;
      }
    }

    if (retry_count == max_retry)
    {
      RCLCPP_ERROR(get_logger(), "Failed to transform pose after %d retries", max_retry);
      return std::nullopt;
    }

    return transformed_pose;
  }

}   // namespace perception
