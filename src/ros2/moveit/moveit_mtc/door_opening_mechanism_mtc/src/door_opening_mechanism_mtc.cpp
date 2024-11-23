#include "door_opening_mechanism_mtc/door_opening_mechanism_mtc.hpp"

namespace door_opening_mechanism_mtc
{

  DoorMechanismMtc::DoorMechanismMtc(const rclcpp::NodeOptions& options) : Node("door_mechanism_mtc", options)
  {
    handle_node_parameter();
    create_subscriptions();
  }

  void DoorMechanismMtc::handle_node_parameter()
  {
    // declare_parameter<std::string>("moveit2_planning_group_name", "mobile_base_arm");
    // declare_parameter<std::string>("planning_pipeline", "ompl_humble");
    // declare_parameter<std::string>("topic_name_pose_stamped", "/stereo/door_handle_pose");
    // declare_parameter<std::string>("topic_name_trigger_door_opening", "/trigger_door_opening");
    // declare_parameter<double>("door_handle_pose_timeout", 2.0);

    // _planning_group_name = get_parameter("moveit2_planning_group_name").as_string();
    // _planning_pipeline = get_parameter("planning_pipeline").as_string();
    // _topic_name_pose_stamped = get_parameter("topic_name_pose_stamped").as_string();
    // _topic_name_trigger_door_opening = get_parameter("topic_name_trigger_door_opening").as_string();
    // _door_handle_pose_timeout = get_parameter("door_handle_pose_timeout").as_double();

    _planning_group_name = "mobile_base_arm";
    _planning_pipeline = "ompl_humble";
    _topic_name_pose_stamped = "/stereo/door_handle_pose";
    _topic_name_trigger_door_opening = "/trigger_door_opening";
    _door_handle_pose_timeout = 2.0;
  }

  void DoorMechanismMtc::create_subscriptions()
  {
    _door_handle_pose_subscription = create_subscription<geometry_msgs::msg::PoseStamped>(
      _topic_name_pose_stamped,
      10,
      std::bind(&DoorMechanismMtc::door_handle_pose_callback, this, std::placeholders::_1));

    _trigger_door_opening_subscription = create_subscription<std_msgs::msg::Empty>(
      _topic_name_trigger_door_opening,
      10,
      std::bind(&DoorMechanismMtc::trigger_door_opening_callback, this, std::placeholders::_1));
  }

  void DoorMechanismMtc::door_handle_pose_callback(const geometry_msgs::msg::PoseStamped& msg)
  {
    RCLCPP_DEBUG(_LOGGER, "Received door handle pose message!");

    _latest_door_handle_pose = msg;
  }

  void DoorMechanismMtc::trigger_door_opening_callback(const std_msgs::msg::Empty& msg)
  {
    RCLCPP_INFO(_LOGGER, "Received trigger door opening message!");

    if (_latest_door_handle_pose.header.stamp.sec == 0)
    {
      RCLCPP_ERROR(_LOGGER, "No door handle pose received yet!");
      return;
    }

    // Check that the latest door handle pose is not too old
    if (now() - _latest_door_handle_pose.header.stamp >
        rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(_door_handle_pose_timeout))))
    {
      RCLCPP_ERROR(_LOGGER, "Latest door handle pose is too old!");
      return;
    }

    // Very important: We spin up the moveit interaction in new thread, otherwise
    // the current state monitor won't get any information about the robot's state.
    std::thread{std::bind(&DoorMechanismMtc::open_door, this)}.detach();
  }

  void DoorMechanismMtc::open_door()
  {
    do_task(_latest_door_handle_pose);
  }

  void DoorMechanismMtc::setup_planning_scene()
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.1, 0.02};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = -0.25;
    pose.orientation.w = 1.0;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
  }

  void DoorMechanismMtc::do_task(const geometry_msgs::msg::PoseStamped target_pose)
  {
    _task = create_task(target_pose);

    RCLCPP_INFO(_LOGGER, "Initializing task!");
    try
    {
      _task.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(_LOGGER, e);
      return;
    }

    RCLCPP_INFO(_LOGGER, "Planning task!");
    try
    {
      if (!_task.plan(5 /* max_solutions */))
      {
        RCLCPP_ERROR(_LOGGER, "Task planning failed");
        return;
      }
    }
    catch (mtc::InitStageException& e)
    {
      std::cerr << e << std::endl;
      throw;
    }

    RCLCPP_INFO(_LOGGER, "Publishing task solutions!");
    _task.introspection().publishSolution(*_task.solutions().front());

    RCLCPP_INFO(_LOGGER, "Executing task!");
    // TODO: Uncomment this line to execute the task
    auto result = _task.execute(*_task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(_LOGGER, "Task execution failed");
      return;
    }

    return;
  }

  mtc::Task DoorMechanismMtc::create_task(geometry_msgs::msg::PoseStamped target_pose)
  {
    RCLCPP_INFO(_LOGGER, "Creating task!");

    RCLCPP_INFO(_LOGGER, "target_pose with frame %s:", target_pose.header.frame_id.c_str());
    RCLCPP_INFO(_LOGGER, "  x: %f", target_pose.pose.position.x);
    RCLCPP_INFO(_LOGGER, "  y: %f", target_pose.pose.position.y);
    RCLCPP_INFO(_LOGGER, "  z: %f", target_pose.pose.position.z);

    mtc::Task task;
    task.stages()->setName("approach door handle task");
    task.loadRobotModel(shared_from_this());

    const auto& group_name = "mobile_base_arm";
    const auto& end_effector_name = "door_opening_end_effector";
    const auto& end_effector_parent_link = "robot/door_opening_mechanism_link_freely_rotating_hook";

    // Set task properties
    task.setProperty("group", group_name);
    task.setProperty("eef", end_effector_name);
    task.setProperty("ik_frame", end_effector_parent_link);

    mtc::Stage* current_state_ptr = nullptr;   // Forward current_state on to grasp pose generator
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    // Setup the planner
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this(), _planning_pipeline);
    moveit_msgs::msg::WorkspaceParameters workspace_params;
    workspace_params.min_corner.x = -10.0;
    workspace_params.min_corner.y = -10.0;
    workspace_params.min_corner.z = -10.0;
    workspace_params.max_corner.x = 10.0;
    workspace_params.max_corner.y = 10.0;
    workspace_params.max_corner.z = 10.0;
    sampling_planner->setProperty("workspace_parameters", workspace_params);

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    auto modify_planning_scene_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Disable collision checking");
    modify_planning_scene_stage->allowCollisions("robot/right_wheel_link", "robot/right_wheel_tire_link", true);
    modify_planning_scene_stage->allowCollisions("robot/left_wheel_link", "robot/left_wheel_tire_link", true);
    task.add(std::move(modify_planning_scene_stage));

    auto stage = std::make_unique<mtc::stages::MoveTo>("Starting position", sampling_planner);
    stage->setGroup(group_name);
    stage->setGoal("starting_position_middle");
    stage->setProperty("collision_check", false);   // Disable collision checking
    task.add(std::move(stage));

    // Connect the initial stage with the generated IK solution using the sampling planner.
    // auto connector_stage = std::make_unique<moveit::task_constructor::stages::Connect>(
    //   "Connect", moveit::task_constructor::stages::Connect::GroupPlannerVector{{group_name, sampling_planner}});
    // connector_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"eef", "group"});
    // task.add(std::move(connector_stage));

    // Generate a pose (this gets put in the IK wrapper below)
    auto generate_pose_stage = std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate pose");
    generate_pose_stage->setPose(target_pose);
    generate_pose_stage->setMonitoredStage(task.stages()->findChild("current"));

    // Compute IK
    // auto ik_wrapper =
    //   std::make_unique<moveit::task_constructor::stages::ComputeIK>("generate pose IK",
    //   std::move(generate_pose_stage));
    // ik_wrapper->setMaxIKSolutions(5);
    // ik_wrapper->setMinSolutionDistance(1.0);
    // ik_wrapper->setTimeout(1.0);
    // // ik_wrapper->setIKFrame(end_effector_parent_link);
    // ik_wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"eef", "group"});
    // ik_wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, {"target_pose"});
    // task.add(std::move(ik_wrapper));

    return task;
  }

}   // namespace door_opening_mechanism_mtc
