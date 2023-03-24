#include "gazebo_controller_manager/joint_position_controller.hpp"

namespace gazebo_controller_manager
{
  JointPositionController::JointPositionController(const rclcpp::Node::SharedPtr& nh,
                                                   const std::vector<std::string>& joint_names,
                                                   const std::string& ros_joint_trajectory_topic)
  {
    std::vector<std::string> gz_cmd_topics = this->get_gz_cmd_joint_topics(joint_names);

    // ROS and gz node
    this->nh_ = nh;
    this->gz_node_ = std::make_shared<gz::transport::Node>();
    // check
    if (joint_names.size() != gz_cmd_topics.size())
    {
      RCLCPP_ERROR(this->nh_->get_logger(), "The size of the arrays joint_names and gz_cmd_topics are not matched!");
      return;
    }
    joint_names_ = joint_names;
    // init joint names map
    for (size_t i = 0; i < joint_names_.size(); i++)
    {
      joint_names_map_[joint_names_[i]] = i;
    }
    // create ros pub and sub
    ros_joint_trajectory_sub_ = nh_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        ros_joint_trajectory_topic,
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

  void JointPositionController::set_joint_position_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    for (auto i = 0u; i < msg->points[0].positions.size(); ++i)
    {
      if (joint_names_map_.find(msg->joint_names[i]) != joint_names_map_.end())
      {
        // find joint name in `joint_names_` .
        int idx = joint_names_map_[msg->joint_names[i]];
        gz::msgs::Double ign_msg;
        ign_msg.set_data(msg->points[0].positions[i]);
        gz_cmd_joint_pubs_[idx]->Publish(ign_msg);
      }
    }
  }
}   // namespace gazebo_controller_manager