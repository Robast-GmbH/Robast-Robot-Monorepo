#ifndef ZED_BRIDGE__PERSON_DISTANCE_PUB_HPP_
#define ZED_BRIDGE__PERSON_DISTANCE_PUB_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>

class PersonDistanceNode : public rclcpp::Node
{
 public:
  PersonDistanceNode();

 private:
  void topic_callback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg);

  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string _target_frame;
};

#endif   // ZED_BRIDGE__PERSON_DISTANCE_PUB_HPP_