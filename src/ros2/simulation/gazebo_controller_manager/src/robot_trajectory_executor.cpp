#include "gazebo_controller_manager/robot_trajectory_executor.hpp"

namespace gazebo_controller_manager
{
  RobotTrajectoryExecutor::RobotTrajectoryExecutor(const rclcpp::Node::SharedPtr& node,
                                                   const std::vector<std::string>& joint_names,
                                                   const std::string& ros_robot_trajectory_topic)
  {
    RCLCPP_INFO(node->get_logger(), "Starting RobotTrajectoryExecutor!");

    this->node_ = node;

    this->cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    init_joint_names_map(joint_names);

    create_ros_subscriber(ros_robot_trajectory_topic);

    create_gz_publisher(joint_names);
  }

  void RobotTrajectoryExecutor::create_gz_publisher(const std::vector<std::string>& joint_names)
  {
    this->gz_node_ = std::make_shared<gz::transport::Node>();

    std::vector<std::string> gz_cmd_topics = this->get_gz_cmd_joint_topics(joint_names);

    if (joint_names.size() != gz_cmd_topics.size())
    {
      RCLCPP_ERROR(this->node_->get_logger(), "The size of the arrays joint_names and gz_cmd_topics are not matched!");
      return;
    }

    // create gz pub
    for (size_t i = 0; i < gz_cmd_topics.size(); i++)
    {
      auto pub =
          std::make_shared<gz::transport::Node::Publisher>(gz_node_->Advertise<gz::msgs::Double>(gz_cmd_topics[i]));
      gz_cmd_joint_pubs_.push_back(pub);
    }
  }

  void RobotTrajectoryExecutor::create_ros_subscriber(const std::string& ros_robot_trajectory_topic)
  {
    ros_robot_trajectory_sub_ = node_->create_subscription<moveit_msgs::msg::RobotTrajectory>(
        ros_robot_trajectory_topic,
        10,
        std::bind(&RobotTrajectoryExecutor::execute_robot_trajectory_cb, this, std::placeholders::_1));
  }

  void RobotTrajectoryExecutor::init_joint_names_map(const std::vector<std::string>& joint_names)
  {
    this->joint_names_ = joint_names;

    for (size_t i = 0; i < joint_names_.size(); i++)
    {
      joint_names_map_[joint_names_[i]] = i;
    }
  }

  std::vector<std::string> RobotTrajectoryExecutor::get_gz_cmd_joint_topics(std::vector<std::string> joint_names)
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

  void RobotTrajectoryExecutor::execute_robot_trajectory_cb(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg)
  {
    // print_robot_trajectory_msg(msg);

    set_single_dof_joint_trajectory(msg->joint_trajectory);

    set_multi_dof_joint_trajectory(msg->multi_dof_joint_trajectory);   // mobile base = planar joint
  }

  void RobotTrajectoryExecutor::set_single_dof_joint_trajectory(const trajectory_msgs::msg::JointTrajectory& msg)
  {
    if (msg.points.size() > 0)
    {
      for (auto i = 0u; i < msg.points[0].positions.size(); ++i)
      {
        if (joint_names_map_.find(msg.joint_names[i]) != joint_names_map_.end())
        {
          // find joint name in `joint_names_` .
          int idx = joint_names_map_[msg.joint_names[i]];
          gz::msgs::Double ign_msg;
          ign_msg.set_data(msg.points[0].positions[i]);
          gz_cmd_joint_pubs_[idx]->Publish(ign_msg);
        }
      }
    }
  }

  void RobotTrajectoryExecutor::set_multi_dof_joint_trajectory(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg)
  {
    auto cmd_vel_msg = std::make_shared<geometry_msgs::msg::Twist>();

    if (msg.points.size() > 0 && msg.points[0].velocities.size() > 0)
    {
      cmd_vel_msg->linear.x = msg.points[0].velocities[0].linear.x;
      cmd_vel_msg->linear.y = msg.points[0].velocities[0].linear.y;
      cmd_vel_msg->linear.z = msg.points[0].velocities[0].linear.z;
      cmd_vel_msg->angular.x = msg.points[0].velocities[0].angular.x;
      cmd_vel_msg->angular.y = msg.points[0].velocities[0].angular.y;
      cmd_vel_msg->angular.z = msg.points[0].velocities[0].angular.z;
    }
    else
    {
      cmd_vel_msg->linear.x = 0;
      cmd_vel_msg->linear.y = 0;
      cmd_vel_msg->linear.z = 0;
      cmd_vel_msg->angular.x = 0;
      cmd_vel_msg->angular.y = 0;
      cmd_vel_msg->angular.z = 0;
    }

    cmd_vel_publisher_->publish(*cmd_vel_msg);
  }

  void RobotTrajectoryExecutor::print_robot_trajectory_msg(const moveit_msgs::msg::RobotTrajectory::SharedPtr& msg)
  {
    if (!msg)
    {
      RCLCPP_WARN(this->node_->get_logger(), "Empty joint trajectory message received");
      return;
    }

    RCLCPP_INFO(this->node_->get_logger(), "-----------------");

    print_joint_trajectory_msg(msg->joint_trajectory);

    print_multi_dof_joint_trajectory(msg->multi_dof_joint_trajectory);
  }

  void RobotTrajectoryExecutor::print_joint_trajectory_msg(const trajectory_msgs::msg::JointTrajectory& msg)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Joint Trajectory:");

    // Print the header information
    RCLCPP_INFO(this->node_->get_logger(), "Header:");
    RCLCPP_INFO(this->node_->get_logger(), "  stamp: %f", msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec);
    RCLCPP_INFO(this->node_->get_logger(), "  frame_id: %s", msg.header.frame_id.c_str());

    // Print the joint names
    RCLCPP_INFO(this->node_->get_logger(), "Joint Names:");
    for (const auto& name : msg.joint_names)
    {
      RCLCPP_INFO(this->node_->get_logger(), "  %s", name.c_str());
    }

    // Print the points in the trajectory
    RCLCPP_INFO(this->node_->get_logger(), "Trajectory Points (number of points: %zu):", msg.points.size());
    for (size_t i = 0; i < msg.points.size(); ++i)
    {
      const auto& point = msg.points[i];

      RCLCPP_INFO(this->node_->get_logger(), "  Point %zu:", i);
      RCLCPP_INFO(this->node_->get_logger(),
                  "    time_from_start: %f",
                  point.time_from_start.sec + 1e-9 * point.time_from_start.nanosec);

      RCLCPP_INFO(this->node_->get_logger(), "    positions:");
      for (const auto& pos : point.positions)
      {
        RCLCPP_INFO(this->node_->get_logger(), "      %f", pos);
      }

      RCLCPP_INFO(this->node_->get_logger(), "    velocities:");
      for (const auto& vel : point.velocities)
      {
        RCLCPP_INFO(this->node_->get_logger(), "      %f", vel);
      }

      // RCLCPP_INFO(this->node_->get_logger(), "    accelerations:");
      // for (const auto& accel : point.accelerations)
      // {
      //   RCLCPP_INFO(this->node_->get_logger(), "      %f", accel);
      // }

      // RCLCPP_INFO(this->node_->get_logger(), "    efforts:");
      // for (const auto& effort : point.effort)
      // {
      //   RCLCPP_INFO(this->node_->get_logger(), "      %f", effort);
      // }
    }
  }

  void RobotTrajectoryExecutor::print_multi_dof_joint_trajectory(
      const trajectory_msgs::msg::MultiDOFJointTrajectory& msg)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Multi DOF Joint Trajectory:");

    RCLCPP_INFO(this->node_->get_logger(), "Header: ");
    RCLCPP_INFO(this->node_->get_logger(), "\tstamp: %f", msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec);
    RCLCPP_INFO(this->node_->get_logger(), "\tframe_id: %s", msg.header.frame_id.c_str());

    RCLCPP_INFO(this->node_->get_logger(), "Joint names:");
    for (const auto& joint_name : msg.joint_names)
    {
      RCLCPP_INFO(this->node_->get_logger(), "\t%s", joint_name.c_str());
    }

    RCLCPP_INFO(this->node_->get_logger(), "Points:");
    for (const auto& point : msg.points)
    {
      RCLCPP_INFO(this->node_->get_logger(),
                  "\tTime from start: %f",
                  point.time_from_start.sec + 1e-9 * point.time_from_start.nanosec);
      for (const auto& transform : point.transforms)
      {
        RCLCPP_INFO(this->node_->get_logger(), "\t\tTranslation:");
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\tx: %f", transform.translation.x);
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\ty: %f", transform.translation.y);
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\tz: %f", transform.translation.z);
        RCLCPP_INFO(this->node_->get_logger(), "\t\tRotation:");
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\tw: %f", transform.rotation.w);
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\tx: %f", transform.rotation.x);
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\ty: %f", transform.rotation.y);
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\tz: %f", transform.rotation.z);
      }
      for (const auto& velocity : point.velocities)
      {
        RCLCPP_INFO(this->node_->get_logger(), "\t\tVelocity linear:");
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\tx: %f", velocity.linear.x);
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\ty: %f", velocity.linear.y);
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\tz: %f", velocity.linear.z);
        RCLCPP_INFO(this->node_->get_logger(), "\t\tVelocity angular:");
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\tx: %f", velocity.angular.x);
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\ty: %f", velocity.angular.y);
        RCLCPP_INFO(this->node_->get_logger(), "\t\t\tz: %f", velocity.angular.z);
      }
    }
  }
}   // namespace gazebo_controller_manager