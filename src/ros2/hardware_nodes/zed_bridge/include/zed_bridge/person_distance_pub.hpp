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

  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr _subscription;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _publisher;
  std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  std::string _target_frame;
};

#endif   // ZED_BRIDGE__PERSON_DISTANCE_PUB_HPP_