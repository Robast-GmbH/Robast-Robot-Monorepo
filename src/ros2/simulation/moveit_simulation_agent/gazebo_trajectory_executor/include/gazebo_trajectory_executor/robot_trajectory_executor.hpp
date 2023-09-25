#ifndef RB_THERON__JOINT_POSITION_CONTROLLER_HPP_
#define RB_THERON__JOINT_POSITION_CONTROLLER_HPP_

#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <unordered_map>

namespace gazebo_trajectory_executor
{

  class RobotTrajectoryExecutor
  {
   public:
    RobotTrajectoryExecutor(const rclcpp::Node::SharedPtr& node,
                            const std::vector<std::string>& joint_names,
                            const std::string& ros_robot_trajectory_topic);
    ~RobotTrajectoryExecutor(){};

   private:
    void execute_robot_trajectory_cb(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg);

    void set_single_dof_joint_trajectory(const trajectory_msgs::msg::JointTrajectoryPoint& joint_trajectory_point,
                                         const std::vector<std::string> joint_names);
    void set_multi_dof_joint_trajectory(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& msg);

    std::vector<std::string> get_gz_cmd_joint_topics(std::vector<std::string> joint_names);

    void print_robot_trajectory_msg(const moveit_msgs::msg::RobotTrajectory::SharedPtr& msg);
    void print_joint_trajectory_msg(const trajectory_msgs::msg::JointTrajectory& msg);
    void print_multi_dof_joint_trajectory(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg);

   private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<gz::transport::Node> gz_node_;

    rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr ros_robot_trajectory_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    std::vector<std::shared_ptr<gz::transport::Node::Publisher>> gz_cmd_joint_pubs_;
    // joint names and map
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, int> joint_names_map_;

    void create_gz_publisher(const std::vector<std::string>& joint_names);

    void create_ros_subscriber(const std::string& ros_robot_trajectory_topic);

    void init_joint_names_map(const std::vector<std::string>& joint_names);
  };
}   // namespace gazebo_trajectory_executor
#endif   // RB_THERON__JOINT_POSITION_CONTROLLER_HPP_