#ifndef RB_THERON__JOINT_POSITION_CONTROLLER_HPP_
#define RB_THERON__JOINT_POSITION_CONTROLLER_HPP_

#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <unordered_map>

namespace gazebo_controller_manager
{

  class JointPositionController
  {
   public:
    JointPositionController(const rclcpp::Node::SharedPtr& nh,
                            const std::vector<std::string>& joint_names,
                            const std::string& ros_cmd_topic);
    ~JointPositionController(){};

   private:
    void set_joint_position_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    std::vector<std::string> get_gz_cmd_joint_topics(std::vector<std::string> joint_names);

   private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<gz::transport::Node> gz_node_;
    // ros pub and sub
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr ros_joint_trajectory_sub_;
    // gz pub
    std::vector<std::shared_ptr<gz::transport::Node::Publisher>> gz_cmd_joint_pubs_;
    // joint names and map
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, int> joint_names_map_;
  };
}   // namespace gazebo_controller_manager
#endif   // RB_THERON__JOINT_POSITION_CONTROLLER_HPP_