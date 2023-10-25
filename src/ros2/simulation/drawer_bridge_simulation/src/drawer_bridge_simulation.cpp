#include "drawer_bridge_simulation/drawer_bridge_simulation.hpp"

namespace drawer_bridge_simulation
{
  DrawerSimulation::DrawerSimulation() : Node("drawer_bridge_simulation")
  {
    RCLCPP_INFO(this->get_logger(), "Creating Drawer Bridge Simulation Node!");

    declare_parameters();

    create_drawer_joint_trajectory_action_client();

    // TODO@Jacob: Change QOS
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    _open_drawer_subscription = this->create_subscription<DrawerAddress>(
        "open_drawer", 10, std::bind(&DrawerSimulation::open_drawer_topic_callback, this, std::placeholders::_1));

    _drawer_status_publisher = this->create_publisher<DrawerStatus>("drawer_is_open", qos);
  }

  void DrawerSimulation::declare_parameters()
  {
    this->declare_parameter("time_until_drawer_closes_automatically_in_ms",
                            _DEFAULT_TIME_UNTIL_DRAWER_CLOSES_AUTOMATICALLY);
    _time_until_drawer_closes_automatically =
        this->get_parameter("time_until_drawer_closes_automatically_in_ms").as_int();

    this->declare_parameter("num_of_drawers", _DEFAULT_NUM_OF_DRAWERS);
    _num_of_drawers = this->get_parameter("num_of_drawers").as_int();
  }

  void DrawerSimulation::create_drawer_joint_trajectory_action_client()
  {
    _drawer_joint_trajectory_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/drawer_joint_trajectory_controller/follow_joint_trajectory");

    bool response = _drawer_joint_trajectory_client->wait_for_action_server(std::chrono::seconds(1));
    if (!response)
    {
      throw std::runtime_error("could not get action server");
    }
    RCLCPP_INFO(this->get_logger(), "Created drawer_joint_trajectory_client successfully!");
  }

  void DrawerSimulation::send_drawer_feedback(DrawerStatus drawer_status_msg, bool drawer_is_open)
  {
    RCLCPP_INFO(this->get_logger(),
                "Sending send_drawer_feedback with module_id: '%i'",
                drawer_status_msg.drawer_address.module_id);
    drawer_status_msg.drawer_is_open = drawer_is_open;
    _drawer_status_publisher->publish(drawer_status_msg);
  }

  std::shared_ptr<rclcpp::Node> DrawerSimulation::get_shared_pointer_of_node()
  {
    return std::enable_shared_from_this<rclcpp::Node>::shared_from_this();
  }

  trajectory_msgs::msg::JointTrajectoryPoint DrawerSimulation::create_trajectory_point(const uint8_t module_id,
                                                                                       const float target_position,
                                                                                       const float time_from_start)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(_num_of_drawers);

    for (uint8_t i = 1; i <= _num_of_drawers; ++i)
    {
      if (i == module_id)
      {
        point.positions[i - 1] = target_position;
      }
      else
      {
        point.positions[i - 1] = 0.0;
      }
    }
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);

    return point;
  }

  control_msgs::action::FollowJointTrajectory_Goal DrawerSimulation::create_trajectory_goal(const uint8_t module_id,
                                                                                            const float target_position)
  {
    std::vector<std::string> joint_names;
    for (uint8_t i = 1; i <= _num_of_drawers; ++i)
    {
      joint_names.push_back("drawer_" + std::to_string(i) + "_joint");
    }

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

    points.push_back(create_trajectory_point(module_id, target_position, _TRAJECTORY_EXECUTION_TIME_IN_SECONDS));

    control_msgs::action::FollowJointTrajectory_Goal goal_msg;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
    goal_msg.trajectory.joint_names = joint_names;
    goal_msg.trajectory.points = points;

    return goal_msg;
  }

  void DrawerSimulation::common_goal_response(
      rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected!");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted!");
    }
  }

  void DrawerSimulation::common_feedback(
      rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
      const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
  {
    RCLCPP_DEBUG(this->get_logger(), "feedback->desired.positions :");
    for (auto &x : feedback->desired.positions)
    {
      RCLCPP_DEBUG(this->get_logger(), "%f\t", x);
    }

    RCLCPP_DEBUG(this->get_logger(), "feedback->desired.velocities :");
    for (auto &x : feedback->desired.velocities)
    {
      RCLCPP_DEBUG(this->get_logger(), "%f\t", x);
    }
  }

  void DrawerSimulation::common_result_response(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult &result,
      const DrawerAddress drawer_address,
      const float target_pose)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
      {
        RCLCPP_INFO(this->get_logger(), "SUCCEEDED result code");
        DrawerStatus drawer_status = DrawerStatus();
        drawer_status.drawer_address = drawer_address;

        bool drawer_is_open = target_pose > 0;
        this->send_drawer_feedback(drawer_status, drawer_is_open);
        break;
      }
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "Unknown result code");
        break;
    }
  }

  void DrawerSimulation::move_drawer_in_simulation_to_target_pose(const DrawerAddress drawer_address,
                                                                  const float target_pose)
  {
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
    opt.goal_response_callback = std::bind(&DrawerSimulation::common_goal_response, this, std::placeholders::_1);
    opt.result_callback =
        std::bind(&DrawerSimulation::common_result_response, this, std::placeholders::_1, drawer_address, target_pose);
    opt.feedback_callback =
        std::bind(&DrawerSimulation::common_feedback, this, std::placeholders::_1, std::placeholders::_2);

    control_msgs::action::FollowJointTrajectory_Goal goal =
        create_trajectory_goal(drawer_address.module_id, target_pose);

    _drawer_joint_trajectory_client->async_send_goal(goal, opt);
  }

  void DrawerSimulation::open_drawer_in_simulation(const DrawerAddress drawer_address, const float target_pose)
  {
    this->move_drawer_in_simulation_to_target_pose(drawer_address, target_pose);
  }

  void DrawerSimulation::async_wait_until_closing_drawer_in_simulation(const int time_until_drawer_closing,
                                                                       const DrawerAddress drawer_address,
                                                                       const float target_pose)
  {
    boost::asio::io_context io;
    boost::asio::steady_timer t(io, boost::asio::chrono::milliseconds(time_until_drawer_closing));
    t.async_wait(
        boost::bind(&DrawerSimulation::move_drawer_in_simulation_to_target_pose, this, drawer_address, target_pose));
    io.run();
  }

  void DrawerSimulation::open_drawer_topic_callback(const DrawerAddress &msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard from open_drawer topic the module_id: '%i'", msg.module_id);

    if (msg.module_id == 0 || msg.drawer_id == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid module_id or drawer_id");
      return;
    }

    DrawerAddress drawer_address = DrawerAddress();
    drawer_address.module_id = msg.module_id;
    drawer_address.drawer_id = msg.drawer_id;

    this->open_drawer_in_simulation(drawer_address, _target_pose_open_drawer);

    this->async_wait_until_closing_drawer_in_simulation(
        _time_until_drawer_closes_automatically, drawer_address, _target_pose_closed_drawer);
  }
}   // namespace drawer_bridge_simulation