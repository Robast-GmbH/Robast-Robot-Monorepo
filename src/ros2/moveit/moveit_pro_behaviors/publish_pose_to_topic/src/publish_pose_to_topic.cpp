#include <publish_pose_to_topic/publish_pose_to_topic.hpp>

namespace publish_pose_to_topic
{

constexpr auto PORT_ID_POSE_STAMPED_IN = "pose_stamped_in";
constexpr auto PORT_ID_TARGET_FRAME_FOR_POSE_STAMPED = "target_frame_pose_stamped";
constexpr auto PORT_ID_TOPIC_NAME = "topic_name";

const std::string TOPIC_POSE_STAMPED = "/debug/pose_stamped";

PublishPoseToTopic::PublishPoseToTopic(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}


BT::PortsList PublishPoseToTopic::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<geometry_msgs::msg::PoseStamped>(PORT_ID_POSE_STAMPED_IN, "{pose_stamped}", "The stamped pose that is to be published."),
      BT::InputPort<std::string>(PORT_ID_TARGET_FRAME_FOR_POSE_STAMPED, "base_footprint", "The target frame for the pose stamped."),
      BT::InputPort<std::string>(PORT_ID_TOPIC_NAME, TOPIC_POSE_STAMPED, "The topic the pose stamped should be published to."),
  });
}

BT::KeyValueVector PublishPoseToTopic::metadata()
{
  return { {"description", "Takes a pose from the blackboard and publishes that pose to a topic. The target frame id can be configured."} };
}

BT::NodeStatus PublishPoseToTopic::onStart()
{
  const auto pose_stamped = getInput<geometry_msgs::msg::PoseStamped>(PORT_ID_POSE_STAMPED_IN);
  const auto taget_frame_pose_stamped = getInput<std::string>(PORT_ID_TARGET_FRAME_FOR_POSE_STAMPED);
  const auto topic_name = getInput<std::string>(PORT_ID_TOPIC_NAME);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(pose_stamped, taget_frame_pose_stamped, topic_name); error)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required values from input data ports." +
                                                                 error.value());
    return BT::NodeStatus::FAILURE;
  }

    // check if the publisher was already created
  if (!_pose_stamped_publisher)
  {    
    _pose_stamped_publisher = shared_resources_->node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name.value(), rclcpp::QoS(1));
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->shared_resources_->node->get_clock());
    _transform_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
  }

  // Transform the pose to the target frame
  geometry_msgs::msg::PoseStamped pose_stamped_transformed = transform_pose(pose_stamped.value(), taget_frame_pose_stamped.value());

  _pose_stamped_publisher->publish(pose_stamped_transformed);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishPoseToTopic::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void PublishPoseToTopic::onHalted()
{
}

 geometry_msgs::msg::PoseStamped PublishPoseToTopic::transform_pose(const geometry_msgs::msg::PoseStamped &pose_stamped, const std::string &target_frame)
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

}  // namespace publish_pose_to_topic
