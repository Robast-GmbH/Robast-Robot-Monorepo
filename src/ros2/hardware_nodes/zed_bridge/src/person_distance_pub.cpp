#include "zed_bridge/person_distance_pub.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

PersonDistanceNode::PersonDistanceNode() : Node("person_distance_node")
{
  subscription_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
      "/robot/zed/zed_node/body_trk/skeletons",
      10,
      std::bind(&PersonDistanceNode::topic_callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("person_distance", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
      geometry_msgs::msg::PointStamped point_in, point_out;
      point_in.header = msg->header;
      point_in.point.x = object.position[0];
      point_in.point.y = object.position[1];
      point_in.point.z = object.position[2];

      try
      {
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(_target_frame, point_in.header.frame_id, tf2::TimePointZero);

        tf2::doTransform(point_in, point_out, transform_stamped);

        float distance =
            std::sqrt(std::pow(point_out.point.x, 2) + std::pow(point_out.point.y, 2) + std::pow(point_out.point.z, 2));

        if (distance < min_distance)
        {
          min_distance = distance;
          closest_point = point_out;
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
  publisher_->publish(closest_point);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PersonDistanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}