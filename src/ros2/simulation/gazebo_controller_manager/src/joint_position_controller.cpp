#include "gazebo_controller_manager/joint_position_controller.hpp"

namespace gazebo_controller_manager
{
  JointPositionController::JointPositionController(const rclcpp::Node::SharedPtr& node,
                                                   const std::vector<std::string>& joint_names,
                                                   const std::string& ros_robot_trajectory_topic)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Starting JointPositionController!");

    std::vector<std::string> gz_cmd_topics = this->get_gz_cmd_joint_topics(joint_names);

    // ROS and gz node
    this->node_ = node;
    this->gz_node_ = std::make_shared<gz::transport::Node>();
    // check
    if (joint_names.size() != gz_cmd_topics.size())
    {
      RCLCPP_ERROR(this->node_->get_logger(), "The size of the arrays joint_names and gz_cmd_topics are not matched!");
      return;
    }
    joint_names_ = joint_names;
    // init joint names map
    for (size_t i = 0; i < joint_names_.size(); i++)
    {
      joint_names_map_[joint_names_[i]] = i;
    }
    // create ros pub and sub
    ros_robot_trajectory_sub_ = node_->create_subscription<moveit_msgs::msg::RobotTrajectory>(
        ros_robot_trajectory_topic,
        10,
        std::bind(&JointPositionController::set_joint_position_cb, this, std::placeholders::_1));
    // create gz pub
    for (size_t i = 0; i < gz_cmd_topics.size(); i++)
    {
      auto pub =
          std::make_shared<gz::transport::Node::Publisher>(gz_node_->Advertise<gz::msgs::Double>(gz_cmd_topics[i]));
      gz_cmd_joint_pubs_.push_back(pub);
    }
  }

  std::vector<std::string> JointPositionController::get_gz_cmd_joint_topics(std::vector<std::string> joint_names)
  {
    std::vector<std::string> gz_cmd_topics;
    for (std::string& joint_name : joint_names)
    {
      std::stringstream ss;
      ss << "/model/rb_theron/joint/" << joint_name << "/0/cmd_pos";
      gz_cmd_topics.push_back(ss.str());
    }
    return gz_cmd_topics;
  }

  void JointPositionController::set_joint_position_cb(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg)
  {
    printJointTrajectory(msg);

    for (auto i = 0u; i < msg->joint_trajectory.points[0].positions.size(); ++i)
    {
      if (joint_names_map_.find(msg->joint_trajectory.joint_names[i]) != joint_names_map_.end())
      {
        // find joint name in `joint_names_` .
        int idx = joint_names_map_[msg->joint_trajectory.joint_names[i]];
        gz::msgs::Double ign_msg;
        ign_msg.set_data(msg->joint_trajectory.points[0].positions[i]);
        gz_cmd_joint_pubs_[idx]->Publish(ign_msg);
      }
    }
  }

  void JointPositionController::printJointTrajectory(const moveit_msgs::msg::RobotTrajectory::SharedPtr& msg)
  {
    if (!msg)
    {
      RCLCPP_WARN(this->node_->get_logger(), "Empty joint trajectory message received");
      return;
    }

    RCLCPP_INFO(this->node_->get_logger(), "Joint Trajectory:");
    RCLCPP_INFO(this->node_->get_logger(), "-----------------");

    // Print the header information
    RCLCPP_INFO(this->node_->get_logger(), "Header:");
    RCLCPP_INFO(this->node_->get_logger(),
                "  stamp: %f",
                msg->joint_trajectory.header.stamp.sec + 1e-9 * msg->joint_trajectory.header.stamp.nanosec);
    RCLCPP_INFO(this->node_->get_logger(), "  frame_id: %s", msg->joint_trajectory.header.frame_id.c_str());

    // Print the joint names
    RCLCPP_INFO(this->node_->get_logger(), "Joint Names:");
    for (const auto& name : msg->joint_trajectory.joint_names)
    {
      RCLCPP_INFO(this->node_->get_logger(), "  %s", name.c_str());
    }

    // Print the points in the trajectory
    RCLCPP_INFO(
        this->node_->get_logger(), "Trajectory Points (number of points: %zu):", msg->joint_trajectory.points.size());
    for (size_t i = 0; i < msg->joint_trajectory.points.size(); ++i)
    {
      const auto& point = msg->joint_trajectory.points[i];

      RCLCPP_INFO(this->node_->get_logger(), "  Point %zu:", i);
      RCLCPP_INFO(this->node_->get_logger(),
                  "    time_from_start: %f",
                  point.time_from_start.sec + 1e-9 * point.time_from_start.nanosec);

      // Print the joint positions
      RCLCPP_INFO(this->node_->get_logger(), "    positions:");
      for (const auto& pos : point.positions)
      {
        RCLCPP_INFO(this->node_->get_logger(), "      %f", pos);
      }

      // Print the joint velocities
      RCLCPP_INFO(this->node_->get_logger(), "    velocities:");
      for (const auto& vel : point.velocities)
      {
        RCLCPP_INFO(this->node_->get_logger(), "      %f", vel);
      }

      // Print the joint accelerations
      RCLCPP_INFO(this->node_->get_logger(), "    accelerations:");
      for (const auto& accel : point.accelerations)
      {
        RCLCPP_INFO(this->node_->get_logger(), "      %f", accel);
      }

      // Print the joint efforts
      RCLCPP_INFO(this->node_->get_logger(), "    efforts:");
      for (const auto& effort : point.effort)
      {
        RCLCPP_INFO(this->node_->get_logger(), "      %f", effort);
      }
    }
  }
}   // namespace gazebo_controller_manager