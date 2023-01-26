#include "ignition_controller_manager/joint_trajectory_controller.hpp"

using namespace std;
using namespace std::placeholders;
using namespace ignition_controller_manager;

JointTrajectoryController::JointTrajectoryController(const rclcpp::NodeOptions & options) : Node("joint_trajectory_controller", options)
{
    // variable
    std::vector<std::string> joint_names;
    const std::vector<std::string> default_joint_names = { "drawer_1_joint", "drawer_2_joint", "drawer_3_joint", "drawer_4_joint", "drawer_5_joint" };
    
    std::vector<std::string> gz_joint_topics;
    const std::vector<std::string> default_gz_joint_topics = { "drawer_1_joint", "drawer_2_joint", "drawer_3_joint", "drawer_4_joint", "drawer_5_joint" };
    
    int update_rate;
    const int default_update_rate = 200;

    std::string follow_joint_trajectory_action;
    const std::string default_follow_joint_trajectory_action = "/planning_group_controller/follow_joint_trajectory";
    
    // Declare parameters
    this->declare_parameter("joint_names", default_joint_names);
    this->declare_parameter("gz_joint_topics", default_gz_joint_topics);
    this->declare_parameter("rate", default_update_rate);
    this->declare_parameter("follow_joint_trajectory_action", default_follow_joint_trajectory_action);

    // Get Parameters
    joint_names = this->get_parameter("joint_names").get_parameter_value().get<std::vector<std::string>>();
    gz_joint_topics = this->get_parameter("gz_joint_topics").get_parameter_value().get<std::vector<std::string>>();
    update_rate = this->get_parameter("rate").as_int();
    follow_joint_trajectory_action = this->get_parameter("follow_joint_trajectory_action").as_string();

    this->initialize_gz_transport_node(joint_names, gz_joint_topics);

    this->action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        follow_joint_trajectory_action,
        std::bind(&JointTrajectoryController::handle_goal, this, _1, _2),
        std::bind(&JointTrajectoryController::handle_cancel, this, _1),
        std::bind(&JointTrajectoryController::handle_accepted, this, _1));

    
    auto period = std::chrono::microseconds(1000000 / update_rate);
    this->update_position_timer_ = this->create_wall_timer(period, std::bind(&JointTrajectoryController::update_position_timer_cb, this));
    //create gz pub
    for (size_t i = 0; i < gz_joint_topics.size(); i++) {
        auto pub = std::make_shared<gz::transport::Node::Publisher>(
            this->gz_transport_node_->Advertise<gz::msgs::Double>(gz_joint_topics[i]));
        this->gz_cmd_joint_pubs_.push_back(pub);
    }
}

void JointTrajectoryController::initialize_gz_transport_node(std::vector<std::string> joint_names, std::vector<std::string> gz_joint_topics)
{
    // ROS and gz node
    this->gz_transport_node_ = std::make_shared<gz::transport::Node>();
    //check
    if (joint_names.size() != gz_joint_topics.size()) {
        RCLCPP_ERROR(this->get_logger(), "The size of arrays are not matched!");
        return;
    }
    this->joint_names_ = joint_names;
    this->joint_num_ = this->joint_names_.size();
    //init joint_names_map_ and target_positions_
    for (size_t i = 0; i < this->joint_num_; i++) {
        this->joint_names_map_[this->joint_names_[i]] = i;
        this->target_positions_.push_back(0);
    }
}

rclcpp_action::GoalResponse JointTrajectoryController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with order");
  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "handle_accepted and executing");

  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&JointTrajectoryController::execute, this, _1), goal_handle}.detach();
}

void JointTrajectoryController::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  const auto goal = goal_handle->get_goal();

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg = goal->trajectory;

  this->set_joint_trajectory_cb(std::make_shared<trajectory_msgs::msg::JointTrajectory>(trajectory_msg));

  if (rclcpp::ok())
  {
    std::unique_lock<std::mutex> lock_until_trajectory_motion_is_finished(this->cv_mutex_);
    this->cv_.wait(
      lock_until_trajectory_motion_is_finished, [this]
      {
        // this block behaves like:
        // while (!is_trajectory_motion_finished()) wait(lck);
        return this->is_trajectory_motion_finished();
      });
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }
}

bool JointTrajectoryController::is_trajectory_motion_finished()
{
    return !this->has_trajectory_;
}

void JointTrajectoryController::update_position_timer_cb()
{
    std::lock_guard<std::mutex> lock(this->trajectory_mutex_);
    //wait trajectory
    if (!this->has_trajectory_) {
        return;
    }
    //check index of trajectory points
    if (trajectory_index_ >= points_.size())
    {
        std::lock_guard<std::mutex> scoped_lock(this->cv_mutex_); // the lock will be released after the scope of this function
        this->has_trajectory_ = false;
        this->cv_.notify_one();
        return;
    }
    // trajectory roll-out based on time, this is not the most efficient way to set things
    // joint position interpolation (simple linear interpolation)
    if (trajectory_index_ < points_.size() - 1) {
        rclcpp::Time current_time = rclcpp::Clock().now();
        rclcpp::Duration cur_time_from_start = current_time - trajectory_start_time_;
        rclcpp::Duration time_from_start = points_[trajectory_index_].time_from_start;
        rclcpp::Duration next_time_from_start = points_[trajectory_index_ + 1].time_from_start;
        double c = (cur_time_from_start - time_from_start).seconds() / (next_time_from_start - time_from_start).seconds();
        c = c > 1 ? 1 : c;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            target_positions_[i] = (1 - c) * points_[trajectory_index_].positions[i] + c * points_[trajectory_index_ + 1].positions[i];
        }
        // increment to next trajectory point
        if (cur_time_from_start >= next_time_from_start) {
            trajectory_index_++;
        }
    } else {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            target_positions_[i] = points_[trajectory_index_].positions[i];
        }
        trajectory_index_++;
    }
    //publish control msg to gz
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        gz::msgs::Double ign_msg;
        ign_msg.set_data(target_positions_[i]);
        gz_cmd_joint_pubs_[i]->Publish(ign_msg);
    }
}

void JointTrajectoryController::set_joint_trajectory_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{   
    //check
    if (msg->joint_names.size() < joint_names_.size())
    {
        return;
    }
    for (size_t k = 0; k < joint_names_.size(); k++) {
        if (joint_names_[k] != msg->joint_names[k])
        {
            return;
        }
    }
    //get points
    {
        std::lock_guard<std::mutex> lock(this->trajectory_mutex_);

        auto chain_size = static_cast<unsigned int>(joint_names_.size());
        auto points_size = static_cast<unsigned int>(msg->points.size());
        std::cout<<"get trajectory msg:"<<points_size<<std::endl; //DEBUGGING
        points_.resize(points_size);
        for (unsigned int i = 0; i < points_size; ++i) {
            points_[i].positions.resize(chain_size);
            points_[i].time_from_start = msg->points[i].time_from_start;
            for (unsigned int j = 0; j < chain_size; ++j) {
                points_[i].positions[j] = msg->points[i].positions[j];
            }
        }
        // trajectory start time
        trajectory_start_time_ = rclcpp::Clock().now();
        this->has_trajectory_ = true;
        this->trajectory_index_ = 0;
    }
}