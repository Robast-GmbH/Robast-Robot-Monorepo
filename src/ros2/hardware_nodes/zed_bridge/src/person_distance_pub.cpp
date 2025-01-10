#include "zed_bridge/person_distance_pub.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

PersonDistanceNode::PersonDistanceNode() : Node("person_distance_node")
{
  _subscription = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
      "/robot/zed/zed_node/body_trk/skeletons",
      10,
      std::bind(&PersonDistanceNode::topic_callback, this, std::placeholders::_1));

  _publisher = create_publisher<geometry_msgs::msg::PointStamped>("person_distance", 10);

  _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
  this->get_parameter_or("target_frame", _target_frame, std::string("robot/base_footprint"));
}

void PersonDistanceNode::topic_callback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg)
{
  geometry_msgs::msg::PointStamped closest_point;
  if (msg->objects.empty())
  {
    RCLCPP_DEBUG(this->get_logger(), "No objects detected.");
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "Detected %ld objects.", msg->objects.size());
    float min_distance = std::numeric_limits<float>::max();
    for (const auto &object : msg->objects)
    {
      geometry_msgs::msg::PointStamped detected_point, transformed_point;
      detected_point.header = msg->header;
      detected_point.point.x = object.position[0];
      detected_point.point.y = object.position[1];
      detected_point.point.z = object.position[2];

      try
      {
        const geometry_msgs::msg::TransformStamped transform_stamped =
            _tf_buffer->lookupTransform(_target_frame, detected_point.header.frame_id, tf2::TimePointZero);

        tf2::doTransform(detected_point, transformed_point, transform_stamped);

        const float distance =
            std::sqrt(std::pow(transformed_point.point.x, 2) + std::pow(transformed_point.point.y, 2) +
                      std::pow(transformed_point.point.z, 2));

        if (distance < min_distance)
        {
          min_distance = distance;
          closest_point = transformed_point;
        }
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s", ex.what());
        return;
      }
    }
  }
  closest_point.header.stamp = this->get_clock()->now();
  _publisher->publish(closest_point);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PersonDistanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}