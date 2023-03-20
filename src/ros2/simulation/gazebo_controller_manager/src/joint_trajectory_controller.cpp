#include "gazebo_controller_manager/joint_trajectory_controller.hpp"

namespace gazebo_controller_manager
{
  JointTrajectoryController::JointTrajectoryController(const rclcpp::NodeOptions& options)
      : Node("joint_trajectory_controller", options)
  {
    // variable
    std::vector<std::string> joint_names;
    const std::vector<std::string> default_joint_names = {
        "drawer_1_joint", "drawer_2_joint", "drawer_3_joint", "drawer_4_joint", "drawer_5_joint"};

    int update_rate;
    const int default_update_rate = 200;

    std::string follow_joint_trajectory_action;
    const std::string default_follow_joint_trajectory_action = "/planning_group_controller/follow_joint_trajectory";

    // Declare parameters
    this->declare_parameter("joint_names", default_joint_names);
    this->declare_parameter("rate", default_update_rate);
    this->declare_parameter("follow_joint_trajectory_action", default_follow_joint_trajectory_action);

    // Get Parameters
    joint_names = this->get_parameter("joint_names").get_parameter_value().get<std::vector<std::string>>();
    update_rate = this->get_parameter("rate").as_int();
    follow_joint_trajectory_action = this->get_parameter("follow_joint_trajectory_action").as_string();

    std::vector<std::string> gz_cmd_topics = this->get_gz_cmd_joint_topics(joint_names);

    this->initialize_gz_transport_node(joint_names, gz_cmd_topics);

    this->follow_joint_trajectory_action_server_ =
        rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            follow_joint_trajectory_action,
            std::bind(&JointTrajectoryController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&JointTrajectoryController::handle_cancel, this, std::placeholders::_1),
            std::bind(&JointTrajectoryController::handle_accepted, this, std::placeholders::_1));

    this->execute_trajectory_action_server_ = rclcpp_action::create_server<moveit_msgs::action::ExecuteTrajectory>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/execute_trajectory",
        std::bind(&JointTrajectoryController::handle_execute_trajectory_goal,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&JointTrajectoryController::handle_execute_trajectory_cancel, this, std::placeholders::_1),
        std::bind(&JointTrajectoryController::handle_execute_trajectory_accepted, this, std::placeholders::_1));

    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    auto period = std::chrono::microseconds(1000000 / update_rate);
    this->update_position_timer_ =
        this->create_wall_timer(period, std::bind(&JointTrajectoryController::update_joint_position_timer_cb, this));
    // create gz pub
    for (size_t i = 0; i < gz_cmd_topics.size(); i++)
    {
      auto pub = std::make_shared<gz::transport::Node::Publisher>(
          this->gz_transport_node_->Advertise<gz::msgs::Double>(gz_cmd_topics[i]));
      this->gz_cmd_joint_pubs_.push_back(pub);
    }
  }

  std::vector<std::string> JointTrajectoryController::get_gz_cmd_joint_topics(std::vector<std::string> joint_names)
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

  void JointTrajectoryController::initialize_gz_transport_node(std::vector<std::string> joint_names,
                                                               std::vector<std::string> gz_cmd_topics)
  {
    // ROS and gz node
    this->gz_transport_node_ = std::make_shared<gz::transport::Node>();
    // check
    if (joint_names.size() != gz_cmd_topics.size())
    {
      RCLCPP_ERROR(this->get_logger(), "The size of the arrays joint_names and gz_cmd_topics are not matched!");
      return;
    }
    this->joint_names_ = joint_names;
    this->joint_num_ = this->joint_names_.size();
    // init joint_names_map_ and target_positions_
    for (size_t i = 0; i < this->joint_num_; i++)
    {
      this->joint_names_map_[this->joint_names_[i]] = i;
      this->target_joint_positions_.push_back(0);
    }
  }

  rclcpp_action::GoalResponse JointTrajectoryController::handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request for follow_joint_trajectory_action_server_");
    (void) uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse JointTrajectoryController::handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal for follow_joint_trajectory_action_server_");
    (void) goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void JointTrajectoryController::handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "handle_accepted and executing for follow_joint_trajectory_action_server_");

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&JointTrajectoryController::execute_follow_joint_trajectory, this, std::placeholders::_1),
                goal_handle}
        .detach();
  }

  rclcpp_action::GoalResponse JointTrajectoryController::handle_execute_trajectory_goal(
      const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const moveit_msgs::action::ExecuteTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request for /execute_trajectory");
    (void) uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse JointTrajectoryController::handle_execute_trajectory_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal for /execute_trajectory");
    (void) goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void JointTrajectoryController::handle_execute_trajectory_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "handle and executing for /execute_trajectory");

    received_execute_trajectory_ = true;

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&JointTrajectoryController::execute_trajectory, this, std::placeholders::_1), goal_handle}
        .detach();
  }

  void JointTrajectoryController::handle_finished_trajectory_execution()
  {
    received_execute_trajectory_ = false;
    has_trajectory_for_mobile_base_ = false;
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }

  void JointTrajectoryController::execute_trajectory(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> goal_handle)
  {
    auto result = std::make_shared<moveit_msgs::action::ExecuteTrajectory::Result>();

    auto goal_trajectory = goal_handle->get_goal()->trajectory;

    trajectory_msgs::msg::JointTrajectory arm_trajectory_msg = goal_trajectory.joint_trajectory;

    trajectory_msgs::msg::MultiDOFJointTrajectory mobile_base_trajectory_msg =
        goal_trajectory.multi_dof_joint_trajectory;

    this->set_joint_trajectory_cb(std::make_shared<trajectory_msgs::msg::JointTrajectory>(arm_trajectory_msg));

    this->set_mobile_base_trajectory(mobile_base_trajectory_msg);

    if (rclcpp::ok())
    {
      std::unique_lock<std::mutex> lock_until_trajectory_motion_is_finished(this->cv_mutex_);
      this->cv_.wait(lock_until_trajectory_motion_is_finished,
                     [this]
                     {
                       return this->is_trajectory_motion_finished();
                     });
      goal_handle->succeed(result);
      handle_finished_trajectory_execution();
    }
  }

  bool JointTrajectoryController::received_action_from_execute_trajectory()
  {
    // sleep for a small amount of time to give the callback fo the /execute_trajectory action time to be triggered
    // before
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (received_execute_trajectory_)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Already received a trajectory execution request on the /execute_trajectory action so trajectory an "
                  "the follow_joint_trajectory_action is ignored!");
      return true;
    }
    return false;
  }

  void JointTrajectoryController::execute_follow_joint_trajectory(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    if (received_action_from_execute_trajectory())
    {
      return;
    }

    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

    const auto goal = goal_handle->get_goal();

    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg = goal->trajectory;

    this->set_joint_trajectory_cb(std::make_shared<trajectory_msgs::msg::JointTrajectory>(trajectory_msg));

    if (rclcpp::ok())
    {
      std::unique_lock<std::mutex> lock_until_trajectory_motion_is_finished(this->cv_mutex_);
      this->cv_.wait(lock_until_trajectory_motion_is_finished,
                     [this]
                     {
                       return this->is_trajectory_motion_finished();
                     });
      goal_handle->succeed(result);
      handle_finished_trajectory_execution();
    }
  }

  bool JointTrajectoryController::is_trajectory_motion_finished()
  {
    return !this->has_trajectory_;
  }

  void JointTrajectoryController::update_cmd_vel_for_mobile_base(double c)
  {
    if (!has_trajectory_for_mobile_base_)
    {
      return;
    }

    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    if (c == 0)
    {
      auto current_velocity_target = mobile_base_trajectory_points_[trajectory_index_].velocities.front();
      msg->linear.x = current_velocity_target.linear.x;
      msg->linear.y = current_velocity_target.linear.y;
      msg->linear.z = current_velocity_target.linear.z;
      msg->angular.x = current_velocity_target.angular.x;
      msg->angular.y = current_velocity_target.angular.y;
      msg->angular.z = current_velocity_target.angular.z;
    }
    else
    {
      auto current_velocity_target = mobile_base_trajectory_points_[trajectory_index_].velocities.front();
      auto next_velocity_target = mobile_base_trajectory_points_[trajectory_index_ + 1].velocities.front();
      msg->linear.x = (1 - c) * current_velocity_target.linear.x + c * next_velocity_target.linear.x;
      msg->linear.y = (1 - c) * current_velocity_target.linear.y + c * next_velocity_target.linear.y;
      msg->linear.z = (1 - c) * current_velocity_target.linear.z + c * next_velocity_target.linear.z;
      msg->angular.x = (1 - c) * current_velocity_target.angular.x + c * next_velocity_target.angular.x;
      msg->angular.y = (1 - c) * current_velocity_target.angular.y + c * next_velocity_target.angular.y;
      msg->angular.z = (1 - c) * current_velocity_target.angular.z + c * next_velocity_target.angular.z;
    }

    cmd_vel_publisher_->publish(*msg);
  }

  void JointTrajectoryController::update_joint_position_timer_cb()
  {
    std::lock_guard<std::mutex> lock(this->trajectory_mutex_);
    if (!this->has_trajectory_)
    {
      return;
    }
    // check index of trajectory points
    if (trajectory_index_ >= joint_trajectory_points_.size())
    {
      std::lock_guard<std::mutex> scoped_lock(this->cv_mutex_);
      this->has_trajectory_ = false;
      this->cv_.notify_one();
      return;
    }
    // trajectory roll-out based on time, this is not the most efficient way to set things
    // joint position interpolation (simple linear interpolation)
    if (trajectory_index_ < joint_trajectory_points_.size() - 1)
    {
      rclcpp::Time current_time = rclcpp::Clock().now();
      rclcpp::Duration cur_time_from_start = current_time - trajectory_start_time_;
      rclcpp::Duration time_from_start = joint_trajectory_points_[trajectory_index_].time_from_start;
      rclcpp::Duration next_time_from_start = joint_trajectory_points_[trajectory_index_ + 1].time_from_start;
      double c = (cur_time_from_start - time_from_start).seconds() / (next_time_from_start - time_from_start).seconds();
      c = c > 1 ? 1 : c;
      for (size_t i = 0; i < joint_names_.size(); ++i)
      {
        target_joint_positions_[i] = (1 - c) * joint_trajectory_points_[trajectory_index_].positions[i] +
                                     c * joint_trajectory_points_[trajectory_index_ + 1].positions[i];
      }

      update_cmd_vel_for_mobile_base(c);

      // increment to next trajectory point
      if (cur_time_from_start >= next_time_from_start)
      {
        trajectory_index_++;
      }
    }
    else
    {
      for (size_t i = 0; i < joint_names_.size(); ++i)
      {
        target_joint_positions_[i] = joint_trajectory_points_[trajectory_index_].positions[i];
      }
      update_cmd_vel_for_mobile_base(0);
      trajectory_index_++;
    }
    // publish control msg to gz
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      gz::msgs::Double gz_msg;
      gz_msg.set_data(target_joint_positions_[i]);
      gz_cmd_joint_pubs_[i]->Publish(gz_msg);
    }
  }

  bool JointTrajectoryController::is_joint_order_correct(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    for (size_t k = 0; k < joint_names_.size(); k++)
    {
      if (joint_names_[k] != msg->joint_names[k])
      {
        return false;
      }
    }
    return true;
  }

  void JointTrajectoryController::reverse_joint_order()
  {
    std::reverse(joint_names_.begin(), joint_names_.end());
    std::reverse(gz_cmd_joint_pubs_.begin(), gz_cmd_joint_pubs_.end());
  }

  void JointTrajectoryController::set_joint_trajectory_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->joint_names.size() < joint_names_.size())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Size of the joint names of the trajectory message does not match the size of the joint names list "
                   "that were given into the joint trajectory controller!");
      return;
    }
    if (!is_joint_order_correct(msg))
    {
      reverse_joint_order();
      if (!is_joint_order_correct(msg))
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "The joint names of the trajectory message does not match joint names list that were given into the "
            "joint trajectory controller!");
        return;
      }
    }

    // get points
    {
      std::lock_guard<std::mutex> lock(this->trajectory_mutex_);
      auto chain_size = static_cast<unsigned int>(joint_names_.size());
      auto points_size = static_cast<unsigned int>(msg->points.size());
      // std::cout << "get trajectory msg:" << points_size << std::endl;   // DEBUGGING
      joint_trajectory_points_.resize(points_size);
      for (unsigned int i = 0; i < points_size; ++i)
      {
        joint_trajectory_points_[i].positions.resize(chain_size);
        joint_trajectory_points_[i].time_from_start = msg->points[i].time_from_start;
        for (unsigned int j = 0; j < chain_size; ++j)
        {
          joint_trajectory_points_[i].positions[j] = msg->points[i].positions[j];
        }
      }
      // trajectory start time
      trajectory_start_time_ = rclcpp::Clock().now();
      this->has_trajectory_ = true;
      this->trajectory_index_ = 0;
    }
  }

  void JointTrajectoryController::set_mobile_base_trajectory(
      trajectory_msgs::msg::MultiDOFJointTrajectory mobile_base_trajectory)
  {
    uint32_t num_of_trajectory_points = mobile_base_trajectory.points.size();

    if (num_of_trajectory_points > 0)
    {
      mobile_base_trajectory_points_.resize(num_of_trajectory_points);
      mobile_base_trajectory_points_ = mobile_base_trajectory.points;

      has_trajectory_for_mobile_base_ = true;
    }
    else
    {
      has_trajectory_for_mobile_base_ = false;
    }
  }
}   // namespace gazebo_controller_manager