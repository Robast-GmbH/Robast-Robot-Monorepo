#include "drawer_bridge_simulation/drawer_bridge_simulation.hpp"

namespace drawer_bridge_simulation
{
  DrawerSimulation::DrawerSimulation() : Node("drawer_bridge_simulation")
  {
    RCLCPP_INFO(this->get_logger(), "Creating Drawer Bridge Simulation Node!");

    this->declare_parameter("time_until_drawer_closes_automatically_in_ms",
                            this->default_time_until_drawer_closes_automatically_);
    this->time_until_drawer_closes_automatically_ =
        this->get_parameter("time_until_drawer_closes_automatically_in_ms").as_int();

    this->declare_parameter("moveit2_planning_group_name", this->default_moveit2_planning_group_name_);
    this->moveit2_planning_group_name_ = this->get_parameter("moveit2_planning_group_name").as_string();

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    this->open_drawer_subscription_ = this->create_subscription<DrawerAddress>(
        "open_drawer", qos, std::bind(&DrawerSimulation::open_drawer_topic_callback, this, std::placeholders::_1));

    this->drawer_leds_subscription_ = this->create_subscription<DrawerLeds>(
        "drawer_leds", qos, std::bind(&DrawerSimulation::drawer_leds_topic_callback, this, std::placeholders::_1));

    this->drawer_status_publisher_ = this->create_publisher<DrawerStatus>("drawer_is_open", qos);
  }

  void DrawerSimulation::send_drawer_feedback(DrawerStatus drawer_status_msg, bool drawer_is_open)
  {
    RCLCPP_INFO(this->get_logger(),
                "Sending send_drawer_feedback with drawer_controller_id: '%i'",
                drawer_status_msg.drawer_address.drawer_controller_id);
    drawer_status_msg.drawer_is_open = drawer_is_open;
    this->drawer_status_publisher_->publish(drawer_status_msg);
  }

  std::shared_ptr<rclcpp::Node> DrawerSimulation::get_shared_pointer_of_node()
  {
    return std::enable_shared_from_this<rclcpp::Node>::shared_from_this();
  }

  void DrawerSimulation::drawer_leds_topic_callback(const DrawerLeds &msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard from drawer_leds topic the led mode: '%i'", msg.mode);
  }

  void DrawerSimulation::move_drawer_in_simulation_to_target_pose(
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
      const DrawerAddress drawer_address,
      const float target_pose)
  {
    std::string drawer_joint = "drawer_" + std::to_string(drawer_address.drawer_controller_id) + "_joint";

    move_group_interface->setJointValueTarget(drawer_joint, target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [move_group_interface]
    {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface->plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
      auto result = move_group_interface->execute(plan);
      if (result == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Executing Plan succeeded!");

        DrawerStatus drawer_status = DrawerStatus();
        drawer_status.drawer_address = drawer_address;

        bool drawer_is_open = target_pose > 0;
        this->send_drawer_feedback(drawer_status, drawer_is_open);
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }
  }

  void DrawerSimulation::open_drawer_in_simulation(
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
      const DrawerAddress drawer_address,
      const float target_pose)
  {
    this->move_drawer_in_simulation_to_target_pose(move_group_interface, drawer_address, target_pose);
  }

  void DrawerSimulation::async_wait_until_closing_drawer_in_simulation(
      const int time_until_drawer_closing,
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
      const DrawerAddress drawer_address,
      const float target_pose)
  {
    boost::asio::io_context io;
    boost::asio::steady_timer t(io, boost::asio::chrono::milliseconds(time_until_drawer_closing));
    t.async_wait(boost::bind(&DrawerSimulation::move_drawer_in_simulation_to_target_pose,
                             this,
                             move_group_interface,
                             drawer_address,
                             target_pose));
    io.run();
  }

  void DrawerSimulation::open_drawer_topic_callback(const DrawerAddress &msg)
  {
    RCLCPP_INFO(
        this->get_logger(), "I heard from open_drawer topic the drawer_controller_id: '%i'", msg.drawer_controller_id);

    if (msg.drawer_controller_id == 0 || msg.drawer_id == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid drawer_controller_id or drawer_id");
      return;
    }

    DrawerAddress drawer_address = DrawerAddress();
    drawer_address.drawer_controller_id = msg.drawer_controller_id;
    drawer_address.drawer_id = msg.drawer_id;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit::planning_interface::MoveGroupInterface(
            this->get_shared_pointer_of_node(), this->moveit2_planning_group_name_));

    this->open_drawer_in_simulation(move_group_interface, drawer_address, this->target_pose_open_drawer_);

    this->async_wait_until_closing_drawer_in_simulation(this->time_until_drawer_closes_automatically_,
                                                        move_group_interface,
                                                        drawer_address,
                                                        this->target_pose_closed_drawer_);
  }
}   // namespace drawer_bridge_simulation