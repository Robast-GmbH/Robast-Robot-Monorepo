#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace utils
{

  enum Direction
  {
    STANDING,
    LEFT,
    RIGHT,
    FORWARD
  };

  constexpr const char *DirectionToString(Direction dir) throw()
  {
    switch (dir)
    {
    case STANDING:
      return "standing";
    case LEFT:
      return "left";
    case RIGHT:
      return "right";
    case FORWARD:
      return "forward";
    default:
      throw std::invalid_argument("Unimplemented item");
    }
  }

  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat)
  {
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(quat, tf2_quat);
    double yaw = tf2::getYaw(tf2_quat);
    return yaw;
  }

  double calculateYawDifference(const geometry_msgs::msg::Quaternion &start_quat, const geometry_msgs::msg::Quaternion &end_quat)
  {
    double start_yaw = getYawFromQuaternion(start_quat);
    double end_yaw = getYawFromQuaternion(end_quat);

    // Compute the difference in yaw
    double yaw_difference = end_yaw - start_yaw;

    // Normalize the yaw difference to [-pi, pi]
    if (yaw_difference > M_PI)
    {
      yaw_difference -= 2 * M_PI;
    }
    else if (yaw_difference < -M_PI)
    {
      yaw_difference += 2 * M_PI;
    }

    return yaw_difference;
  }

  Direction calculateDirection(const geometry_msgs::msg::Pose &start_pose, const geometry_msgs::msg::Pose &end_pose)
  {
    double angle_z = calculateYawDifference(start_pose.orientation, end_pose.orientation);

    if (angle_z > 0.7)
    {
      return Direction::LEFT;
    }
    else if (angle_z < -0.7)
    {
      return Direction::RIGHT;
    }
    else
    {
      return Direction::FORWARD;
    }
  }
}