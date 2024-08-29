#include <catch2/catch_all.hpp>
#include "bt_plugins/utils/calculate_direction.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

TEST_CASE("getYawFromQuaternion returns correct yaw angle", "[utils]")
{
  geometry_msgs::msg::Quaternion quat;
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = 0.0;
  quat.w = 1.0;

  double expected_yaw = 0.0;
  double actual_yaw = utils::getYawFromQuaternion(quat);

  REQUIRE(actual_yaw == expected_yaw);
}

TEST_CASE("calculateYawDifference returns correct yaw difference", "[utils]")
{
  geometry_msgs::msg::Quaternion start_quat;
  start_quat.x = 0.0;
  start_quat.y = 0.0;
  start_quat.z = 0.0;
  start_quat.w = 1.0;

  geometry_msgs::msg::Quaternion end_quat;
  end_quat.x = 0.0;
  end_quat.y = 0.0;
  end_quat.z = 0.7071068;
  end_quat.w = 0.7071068;

  double expected_yaw_difference = M_PI / 2.0;
  double actual_yaw_difference = utils::calculateYawDifference(start_quat, end_quat);

  REQUIRE(actual_yaw_difference == expected_yaw_difference);
}

TEST_CASE("calculateDirection returns correct left direction", "[utils]")
{
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.0;

  geometry_msgs::msg::Pose end_pose;
  end_pose.orientation.x = 0.0;
  end_pose.orientation.y = 0.0;
  end_pose.orientation.z = 0.7071068;
  end_pose.orientation.w = 0.7071068;
  end_pose.position.x = 1.0;

  std::string expected_direction = "left";
  std::string actual_direction = utils::DirectionToString(utils::calculateDirection(start_pose, end_pose));

  REQUIRE(actual_direction == expected_direction);
}

TEST_CASE("calculateDirection returns correct right direction", "[utils]")
{
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.0;

  geometry_msgs::msg::Pose end_pose;
  end_pose.orientation.x = 0.0;
  end_pose.orientation.y = 0.0;
  end_pose.orientation.z = -0.7071068;
  end_pose.orientation.w = 0.7071068;
  end_pose.position.x = 1.0;

  std::string expected_direction = "right";
  std::string actual_direction = utils::DirectionToString(utils::calculateDirection(start_pose, end_pose));

  REQUIRE(actual_direction == expected_direction);
}

TEST_CASE("calculateDirection returns correct forward direction", "[utils]")
{
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.0;

  geometry_msgs::msg::Pose end_pose;
  end_pose.orientation.x = 0.0;
  end_pose.orientation.y = 0.0;
  end_pose.orientation.z = 0.0;
  end_pose.orientation.w = 1.0;
  end_pose.position.x = 1.0;

  std::string expected_direction = "forward";
  std::string actual_direction = utils::DirectionToString(utils::calculateDirection(start_pose, end_pose));

  REQUIRE(actual_direction == expected_direction);
}

TEST_CASE("calculateDirection returns correct forward edge case", "[utils]")
{
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.0;

  geometry_msgs::msg::Pose end_pose;
  end_pose.orientation.x = 0.0;
  end_pose.orientation.y = 0.0;
  end_pose.orientation.z = -0.258819;
  end_pose.orientation.w = 0.9659258;
  end_pose.position.x = 1.0;

  std::string expected_direction = "forward";
  std::string actual_direction = utils::DirectionToString(utils::calculateDirection(start_pose, end_pose));

  REQUIRE(actual_direction == expected_direction);
}