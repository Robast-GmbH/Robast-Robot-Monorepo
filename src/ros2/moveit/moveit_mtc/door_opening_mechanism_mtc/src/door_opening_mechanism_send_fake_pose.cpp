#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "depthai_ros_msgs/msg/spatial_detection.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"

class MinimalPublisher : public rclcpp::Node
{
 public:
  MinimalPublisher() : Node("fake_pose_publisher")
  {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    publisher_ =
      this->create_publisher<depthai_ros_msgs::msg::SpatialDetectionArray>("stereo/door_handle_position", qos);
    timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback()
  {
    depthai_ros_msgs::msg::SpatialDetection fake_detection;
    fake_detection.position.x = 0.2;    // -0.8
    fake_detection.position.y = 0.4;    // 0.02
    fake_detection.position.z = 0.35;   // 0.8

    auto msg = std::make_shared<depthai_ros_msgs::msg::SpatialDetectionArray>();

    msg->header.frame_id = "rb_theron/base_footprint/back_top_realsense_camera_color_link";
    msg->detections.push_back(fake_detection);

    RCLCPP_INFO(this->get_logger(), "Publishing SpatialDetectionArray message!");
    publisher_->publish(*msg);

    this->timer_->cancel();
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}