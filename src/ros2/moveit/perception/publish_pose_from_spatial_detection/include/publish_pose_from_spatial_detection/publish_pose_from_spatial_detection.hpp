#ifndef PERCEPTION_PUBLISH_POSE_FROM_SPATIAL_DETECTION_HPP
#define PERCEPTION_PUBLISH_POSE_FROM_SPATIAL_DETECTION_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace perception
{
  class PublishPoseFromSpatialDetection : public rclcpp::Node
  {
   public:
    PublishPoseFromSpatialDetection();

   private:
    void setup_subscriptions();
    void setup_publishers();
    void callback_spatial_detections(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg);
    void declare_parameters();
    void get_parameters();

    void handle_spatial_detection(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg);

    std::optional<geometry_msgs::msg::PoseStamped> transform_pose(const geometry_msgs::msg::PoseStamped &pose_stamped,
                                                                  const std::string &target_frame);

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr _spatial_detections_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_pub;

    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _transform_listener;

    bool _use_confidence_threshold;
    double _confidence_threshold;
    bool _invert_y_axis;
    std::vector<double> _target_orientation_in_euler;
    std::string _taget_frame_pose_stamped;

    std::string _topic_name_spatial_detections;
    std::string _topic_name_pose_stamped;
  };

}   // namespace perception

#endif   // PERCEPTION_PUBLISH_POSE_FROM_SPATIAL_DETECTION_HPP