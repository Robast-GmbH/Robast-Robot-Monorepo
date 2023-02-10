#include "door_opening_mechanism_simulation/door_opening_mechanism_simulation.hpp"

namespace door_opening_mechanism_simulation
{
  DoorMechanismSimulation::DoorMechanismSimulation() : Node("door_opening_mechanism_simulation")
  {
    RCLCPP_INFO(this->get_logger(), "Creating Door Opening Mechanism Simulation Node!");

    this->declare_parameter("moveit2_planning_group_name", this->default_moveit2_planning_group_name_);
    this->moveit2_planning_group_name_ = this->get_parameter("moveit2_planning_group_name").as_string();

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    this->open_door_subscription_ = this->create_subscription<DrawerAddress>(
        "open_door", qos, std::bind(&DoorMechanismSimulation::open_door_topic_callback, this, std::placeholders::_1));
  }

  std::shared_ptr<rclcpp::Node> DoorMechanismSimulation::get_shared_pointer_of_node()
  {
    return std::enable_shared_from_this<rclcpp::Node>::shared_from_this();
  }

  void DoorMechanismSimulation::move_robot_in_simulation_to_target_pose(
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface, const float target_pose)
  {
    // ompl_interface::OMPLInterface ompl_interface = ompl_interface::OMPLInterface(
    //     move_group_interface->getRobotModel(), this->get_shared_pointer_of_node(), "ompl");

    // move_group_interface->setJointValueTarget(drawer_joint, target_pose);

    // VISUALIZATION
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface->getCurrentState()->getJointModelGroup(this->moveit2_planning_group_name_);

    moveit_visual_tools::MoveItVisualTools visual_tools(this->get_shared_pointer_of_node(),
                                                        "panda_link0",
                                                        "move_group_tutorial",
                                                        move_group_interface->getRobotModel());

    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script */
    /* via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

    visual_tools.trigger();

    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_interface->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
    std::copy(move_group_interface->getJointModelGroupNames().begin(),
              move_group_interface->getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // Start the demo
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Planning to a Pose goal
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group_interface->setPoseTarget(target_pose1);

    // Create a plan to that target pose
    auto const [success, plan] = [move_group_interface]
    {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface->plan(msg));
      return std::make_pair(ok, msg);
    }();

    RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualize plans
    RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose_Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // // Execute the plan
    // if (success)
    // {
    //   auto result = move_group_interface->execute(plan);
    //   if (result == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    //   {
    //     RCLCPP_INFO(this->get_logger(), "Executing Plan succeeded!");

    //     DrawerStatus drawer_status = DrawerStatus();
    //     drawer_status.drawer_address = drawer_address;

    //     bool drawer_is_open = target_pose > 0;
    //     this->send_drawer_feedback(drawer_status, drawer_is_open);
    //   }
    // }
    // else
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    // }
  }

  void DoorMechanismSimulation::open_door_in_simulation(
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface, const float target_pose)
  {
    this->move_robot_in_simulation_to_target_pose(move_group_interface, target_pose);
  }

  void DoorMechanismSimulation::open_door_topic_callback(const DrawerAddress& msg)
  {
    RCLCPP_INFO(
        this->get_logger(), "I heard from open_drawer topic the drawer_controller_id: '%i'", msg.drawer_controller_id);

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit::planning_interface::MoveGroupInterface(
            this->get_shared_pointer_of_node(), this->moveit2_planning_group_name_));

    this->open_door_in_simulation(move_group_interface, this->target_pose_open_drawer_);
  }
}   // namespace door_opening_mechanism_simulation