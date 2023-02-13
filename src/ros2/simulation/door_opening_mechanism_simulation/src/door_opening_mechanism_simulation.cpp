#include "door_opening_mechanism_simulation/door_opening_mechanism_simulation.hpp"

namespace door_opening_mechanism_simulation
{
  DoorMechanismSimulation::DoorMechanismSimulation() : Node("door_opening_mechanism_simulation")
  {
    RCLCPP_INFO(this->get_logger(), "Creating Door Opening Mechanism Simulation Node!");

    this->declare_parameter("planning_plugin", "ompl_interface/OMPLPlanner");

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

    // move_group_interface->startStateMonitor();

    const std::string PLANNING_GROUP = this->moveit2_planning_group_name_;
    robot_model_loader::RobotModelLoader robot_model_loader(this->get_shared_pointer_of_node(), "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    // Using the
    // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`,
    // we can construct a
    // :moveit_codedir:`PlanningScene<moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h>`
    // that maintains the state of the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // Configure a valid robot state
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!this->get_shared_pointer_of_node()->get_parameter("planning_plugin", planner_plugin_name))
      RCLCPP_FATAL(this->get_logger(), "Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(this->get_logger(), "Exception while creating planning plugin loader %s", ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(
              robot_model, this->get_shared_pointer_of_node(), this->get_shared_pointer_of_node()->get_namespace()))
        RCLCPP_FATAL(this->get_logger(), "Could not initialize planner instance");
      RCLCPP_INFO(this->get_logger(), "Using planning interface '%s'", planner_instance->getDescription().c_str());
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (const auto& cls : classes) ss << cls << " ";
      RCLCPP_ERROR(this->get_logger(),
                   "Exception while loading planner '%s': %s\nAvailable plugins: %s",
                   planner_plugin_name.c_str(),
                   ex.what(),
                   ss.str().c_str());
    }

    moveit::planning_interface::MoveGroupInterface move_group(this->get_shared_pointer_of_node(), PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(
        this->get_shared_pointer_of_node(), "odom", "move_group_tutorial", move_group.getRobotModel());
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();   // clear all old markers
    visual_tools.trigger();

    /* Remote control is an introspection tool that allows users to step through a high level script
       via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();

    /* We can also use visual_tools to wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the arm of the Panda
    // specifying the desired pose of the end-effector as input.
    visual_tools.trigger();
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "/odom";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.4;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // We will create the request as a constraint using a helper function available
    // from the
    // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h>`
    // package.
    moveit_msgs::msg::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

    req.group_name = PLANNING_GROUP;
    req.goal_constraints.push_back(pose_goal);

    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not compute plan successfully");
      return;
    }

    // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
        this->get_shared_pointer_of_node()->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 1);
    moveit_msgs::msg::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher->publish(display_trajectory);

    /* Set the state in the planning scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group,
                                        response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.publishAxisLabeled(pose.pose, "goal_1");
    visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* We can also use visual_tools to wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    // Now, setup a joint space goal
    moveit::core::RobotState goal_state(robot_model);
    std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    moveit_msgs::msg::Constraints joint_goal =
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    // Call the planner and visualize the trajectory
    /* Re-construct the planning context */
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    /* Call the Planner */
    context->solve(res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not compute plan successfully");
      return;
    }
    /* Visualize the trajectory */
    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);

    /* Now you should see two planned trajectories in series*/
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher->publish(display_trajectory);

    /* We will add more goals. But first, set the state in the planning
       scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group,
                                        response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.publishAxisLabeled(pose.pose, "goal_2");
    visual_tools.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /* Now, we go back to the first goal to prepare for orientation constrained planning */
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    res.getMessage(response);

    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher->publish(display_trajectory);

    /* Set the state in the planning scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group,
                                        response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.trigger();

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Adding Path Constraints
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
    /* Let's create a new pose goal */

    pose.pose.position.x = 0.32;
    pose.pose.position.y = -0.25;
    pose.pose.position.z = 0.65;
    pose.pose.orientation.w = 1.0;
    moveit_msgs::msg::Constraints pose_goal_2 =
        kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

    /* Now, let's try to move to this new pose goal*/
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal_2);

    /* But, let's impose a path constraint on the motion.
       Here, we are asking for the end-effector to stay level*/
    geometry_msgs::msg::QuaternionStamped quaternion;
    quaternion.header.frame_id = "panda_link0";
    req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);

    // Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
    // (the workspace of the robot)
    // because of this, we need to specify a bound for the allowed planning volume as well;
    // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
    // but that is not being used in this example).
    // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not
    // done in this volume when planning for the arm; the bounds are only used to determine if the sampled
    // configurations are valid.
    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
        req.workspace_parameters.min_corner.z = -2.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
        req.workspace_parameters.max_corner.z = 2.0;

    // Call the planner and visualize all the plans created so far.
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher->publish(display_trajectory);

    /* Set the state in the planning scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group,
                                        response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.publishAxisLabeled(pose.pose, "goal_3");
    visual_tools.publishText(text_pose, "Orientation Constrained Motion Plan (3)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // END_TUTORIAL
    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to exit the demo");
    planner_instance.reset();

    // xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    // VISUALIZATION
    // const moveit::core::JointModelGroup* joint_model_group =
    //     move_group_interface->getCurrentState()->getJointModelGroup(this->moveit2_planning_group_name_);

    // moveit_visual_tools::MoveItVisualTools visual_tools(this->get_shared_pointer_of_node(),
    //                                                     "base_footprint",
    //                                                     "move_group_tutorial",
    //                                                     move_group_interface->getRobotModel());

    // visual_tools.deleteAllMarkers();

    // // /* Remote control is an introspection tool that allows users to step through a high level script */
    // // /* via buttons and keyboard shortcuts in RViz */
    // visual_tools.loadRemoteControl();

    // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    // text_pose.translation().z() = 1.0;
    // visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rviz_visual_tools::WHITE,
    // rviz_visual_tools::XLARGE);

    // visual_tools.trigger();

    // RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_interface->getPlanningFrame().c_str());
    // RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());
    // RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
    // std::copy(move_group_interface->getJointModelGroupNames().begin(),
    //           move_group_interface->getJointModelGroupNames().end(),
    //           std::ostream_iterator<std::string>(std::cout, ", "));

    // // Start the demo
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // // Planning to a Pose goal
    // geometry_msgs::msg::Pose target_pose1;
    // target_pose1.orientation.w = 1.0;
    // target_pose1.position.x = 0.28;
    // target_pose1.position.y = -0.2;
    // target_pose1.position.z = 0.5;
    // move_group_interface->setPoseTarget(target_pose1);

    // // Create a plan to that target pose
    // auto const [success, plan] = [move_group_interface]
    // {
    //   moveit::planning_interface::MoveGroupInterface::Plan msg;
    //   auto const ok = static_cast<bool>(move_group_interface->plan(msg));
    //   return std::make_pair(ok, msg);
    // }();

    // RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // // Visualize plans
    // RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 as trajectory line");
    // visual_tools.publishAxisLabeled(target_pose1, "pose1");
    // visual_tools.publishText(text_pose, "Pose_Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    // visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

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