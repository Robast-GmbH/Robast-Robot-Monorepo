#include "rb_theron_controller_manager/joint_trajectory_controller.hpp"

using namespace std;
using namespace std::placeholders;
using namespace rb_theron_controller_manager;

JointTrajectoryController::JointTrajectoryController(const rclcpp::NodeOptions & options) : Node("joint_trajectory_controller", options)
{
    // variable
    std::vector<std::string> joint_names;
    std::vector<std::string> default_joint_names = { "first_drawer_joint" };
    std::vector<std::string> ign_joint_topics;
    std::vector<std::string> default_ign_joint_topics = {"first_drawer_joint"};
    int update_rate;
    // parameters
    this->declare_parameter("joint_names", default_joint_names);
    this->declare_parameter("ign_joint_topics", default_ign_joint_topics);
    this->declare_parameter("rate", 200);
    joint_names = this->get_parameter("joint_names").get_parameter_value().get<std::vector<std::string>>();
    ign_joint_topics = this->get_parameter("ign_joint_topics").get_parameter_value().get<std::vector<std::string>>();
    update_rate = this->get_parameter("rate").as_int();
    
    // ROS and Ignition node
    this->ign_node_ = std::make_shared<ignition::transport::Node>();
    //check
    if (joint_names.size() != ign_joint_topics.size()) {
        std::cout << "[JointTrajectoryController ERROR]:the size of arrays are not matched!" << std::endl;
        return;
    }
    this->joint_names_ = joint_names;
    this->joint_num_ = joint_names_.size();
    //init joint_names_map_ and target_positions_
    for (size_t i = 0; i < joint_num_; i++) {
        joint_names_map_[joint_names_[i]] = i;
        target_positions_.push_back(0);
    }

    this->action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/drawer_planning_group_controller/follow_joint_trajectory",
        std::bind(&JointTrajectoryController::handle_goal, this, _1, _2),
        std::bind(&JointTrajectoryController::handle_cancel, this, _1),
        std::bind(&JointTrajectoryController::handle_accepted, this, _1));


    // //create ros pub and sub
    // ros_cmd_joint_trajectory_sub_ = nh_->create_subscription<trajectory_msgs::msg::JointTrajectory>(ros_cmd_topic, 10,
    //     std::bind(&jointTrajectoryController::setJointTrajectoryCb, this, std::placeholders::_1));

    
    auto period = std::chrono::microseconds(1000000 / update_rate);
    update_position_timer_ = this->create_wall_timer(period, std::bind(&JointTrajectoryController::updatePositionTimerCb, this));
    //create ignition pub
    for (size_t i = 0; i < ign_joint_topics.size(); i++) {
        auto pub = std::make_shared<ignition::transport::Node::Publisher>(
            ign_node_->Advertise<ignition::msgs::Double>(ign_joint_topics[i]));
        ign_cmd_joint_pubs_.push_back(pub);
    }
}

rclcpp_action::GoalResponse JointTrajectoryController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with order");
  (void)uuid;

  for(int i = 0; i <  goal->trajectory.points.size(); i++) {
    for(int j = 0; j <  goal->trajectory.points[i].positions.size(); j++) {
      printf("position:\t%.5f\t",  goal->trajectory.points[i].positions[j]);
    }
    printf("\n");
    for(int j = 0; j <  goal->trajectory.points[i].velocities.size(); j++) {
      printf("velocities:\t%.5f\t",  goal->trajectory.points[i].velocities[j]);
    }
    printf("\n");
    printf("time_from_start %u %u\n", goal->trajectory.points[i].time_from_start.sec, goal->trajectory.points[i].time_from_start.nanosec);
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  const auto goal = goal_handle->get_goal();

  //send goals
  bool all_succeed = true;

  // Populate a goal
//   std::vector<hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Goal> vector_goal;
//   for(int i = 0; i < action_clients.size(); i++){
//     vector_goal.push_back(hrim_actuator_rotaryservo_actions::action::GoalJointTrajectory::Goal());
//   }

//   for(unsigned int v = 0; v < vector_goal.size(); v++){
//     vector_goal[v].trajectory.points.resize(goal->trajectory.points.size());
//     for(unsigned int i = 0; i < goal->trajectory.points.size(); i++){
//       vector_goal[v].trajectory.points[i].positions.resize(1);
//       vector_goal[v].trajectory.points[i].velocities.resize(1);
//       vector_goal[v].trajectory.points[i].accelerations.resize(1);

//       vector_goal[v].trajectory.points[i].positions[0] = goal->trajectory.points[i].positions[v];
//       vector_goal[v].trajectory.points[i].velocities[0] = goal->trajectory.points[i].velocities[v];
//       vector_goal[v].trajectory.points[i].accelerations[0] = goal->trajectory.points[i].accelerations[v];
//       vector_goal[v].trajectory.points[i].time_from_start.sec = goal->trajectory.points[i].time_from_start.sec;
//       vector_goal[v].trajectory.points[i].time_from_start.nanosec = goal->trajectory.points[i].time_from_start.nanosec;
//     }

//     double wait_time = (double)(vector_goal[v].trajectory.points[goal->trajectory.points.size()-1].time_from_start.sec) +
//                        (double)(vector_goal[v].trajectory.points[goal->trajectory.points.size()-1].time_from_start.nanosec/1e+9) + 1.0;
//   }

//   for(unsigned int v = 0; v < vector_goal.size(); v++){
//     all_succeed &= action_clients.at(v)->send_goal(vector_goal[v]);
//   }

//   if(!all_succeed){
//     RCLCPP_ERROR(this->get_logger(), "Not all action commands succeeded. Exit.");
//     goal_handle->abort(result);
//     return;
//   }

//   RCLCPP_INFO(this->get_logger(), "All goal commands sent");


//   RCLCPP_INFO(this->get_logger(), "Executing goal");

//   while(true){
//     all_succeed = true;
//     for(unsigned int v = 0; v < vector_goal.size(); v++){
//       all_succeed &= action_clients.at(v)->is_goal_done();
//     }
//     if(all_succeed)
//       break;
//   }

//   // // Check if goal is done
//   if (rclcpp::ok()) {
//     goal_handle->succeed(result);
//     RCLCPP_INFO(this->get_logger(), "Goal Suceeded");
//   }
}

void JointTrajectoryController::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  using namespace std::placeholders;
  RCLCPP_INFO(this->get_logger(), "handle_accepted and executing");

  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&JointTrajectoryController::execute, this, _1), goal_handle}.detach();
}

void JointTrajectoryController::updatePositionTimerCb()
{
    std::lock_guard<std::mutex> lock(trajectory_mut_);
    //wait trajectory
    if (!has_trajectory_) {
        return;
    }
    //check index of trajectory points
    if (trajectory_index_ >= points_.size()) {
        has_trajectory_ = false;
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
    //publish control msg to ignition
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        ignition::msgs::Double ign_msg;
        ign_msg.set_data(target_positions_[i]);
        ign_cmd_joint_pubs_[i]->Publish(ign_msg);
    }
}

void JointTrajectoryController::setJointTrajectoryCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    //check
    if (msg->joint_names.size() < joint_names_.size()) {
        return;
    }
    for (size_t k = 0; k < joint_names_.size(); k++) {
        if (joint_names_[k] != msg->joint_names[k]) {
            return;
        }
    }
    //get points
    {
        std::lock_guard<std::mutex> lock(trajectory_mut_);

        auto chain_size = static_cast<unsigned int>(joint_names_.size());
        auto points_size = static_cast<unsigned int>(msg->points.size());
        //std::cout<<"get trajectory msg:"<<points_size<<std::endl;
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
        has_trajectory_ = true;
        trajectory_index_ = 0;
    }
}