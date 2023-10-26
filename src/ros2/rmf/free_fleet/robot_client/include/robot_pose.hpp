#ifndef ROBOT_CLIENT__ROBOT_POSE_HPP_
#define ROBOT_CLIENT__ROBOT_POSE_HPP_

namespace rmf_robot_client
{
  struct RobotPose
  {
    double x_pose;
    double y_pose;
    double yaw_pose;

    // Constructor for convenience
    RobotPose() : x_pose(0), y_pose(0), yaw_pose(0)
    {
    }

    RobotPose(double x_pose, double y_pose, double yaw_pose) : x_pose(x_pose), y_pose(y_pose), yaw_pose(yaw_pose)
    {
    }
  };
}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__ROBOT_POSE_HPP_
