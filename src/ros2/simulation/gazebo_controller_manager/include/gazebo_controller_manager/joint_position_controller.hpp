#ifndef RB_THERON__JOINT_POSITION_CONTROLLER_HPP_
#define RB_THERON__JOINT_POSITION_CONTROLLER_HPP_

#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

namespace gazebo_controller_manager
{

  class JointPositionController
  {
   public:
    JointPositionController(const rclcpp::Node::SharedPtr& node,
                            const std::vector<std::string>& joint_names,
                            const std::string& ros_robot_trajectory_topic);
    ~JointPositionController(){};

   private:
    void set_joint_position_cb(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg);
    std::vector<std::string> get_gz_cmd_joint_topics(std::vector<std::string> joint_names);

    void printJointTrajectory(const moveit_msgs::msg::RobotTrajectory::SharedPtr& msg);

   private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<gz::transport::Node> gz_node_;
    // ros pub and sub
    rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr ros_robot_trajectory_sub_;
    // gz pub
    std::vector<std::shared_ptr<gz::transport::Node::Publisher>> gz_cmd_joint_pubs_;
    // joint names and map
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, int> joint_names_map_;
  };
}   // namespace gazebo_controller_manager
#endif   // RB_THERON__JOINT_POSITION_CONTROLLER_HPP_