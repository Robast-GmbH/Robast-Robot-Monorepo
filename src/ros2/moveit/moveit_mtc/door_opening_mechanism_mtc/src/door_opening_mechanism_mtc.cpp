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
    declare_parameter<std::string>("topic_name_pose_stamped", "/stereo/door_handle_pose");
    declare_parameter<std::string>("topic_name_trigger_door_opening", "/trigger_door_opening");
    declare_parameter<double>("door_handle_pose_timeout", 2.0);

    _planning_group_name = get_parameter("moveit2_planning_group_name").as_string();
    _planning_pipeline = get_parameter("planning_pipeline").as_string();
    _topic_name_pose_stamped = get_parameter("topic_name_pose_stamped").as_string();
    _topic_name_trigger_door_opening = get_parameter("topic_name_trigger_door_opening").as_string();
    _door_handle_pose_timeout = get_parameter("door_handle_pose_timeout").as_double();

    RCLCPP_INFO(_LOGGER, "Node parameters:");
    RCLCPP_INFO(_LOGGER, "  moveit2_planning_group_name: %s", _planning_group_name.c_str());
    RCLCPP_INFO(_LOGGER, "  planning_pipeline: %s", _planning_pipeline.c_str());
    RCLCPP_INFO(_LOGGER, "  topic_name_pose_stamped: %s", _topic_name_pose_stamped.c_str());
    RCLCPP_INFO(_LOGGER, "  topic_name_trigger_door_opening: %s", _topic_name_trigger_door_opening.c_str());
    RCLCPP_INFO(_LOGGER, "  door_handle_pose_timeout: %f", _door_handle_pose_timeout);
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

  void DoorMechanismMtc::do_task(const geometry_msgs::msg::PoseStamped& door_handle_pose)
  {
    _task = create_task(door_handle_pose);

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

  mtc::Task DoorMechanismMtc::create_task(const geometry_msgs::msg::PoseStamped& door_handle_pose)
  {
    RCLCPP_INFO(_LOGGER, "Creating task!");

    RCLCPP_INFO(_LOGGER, "door_handle_pose with frame %s:", door_handle_pose.header.frame_id.c_str());
    RCLCPP_INFO(_LOGGER, "  x: %f", door_handle_pose.pose.position.x);
    RCLCPP_INFO(_LOGGER, "  y: %f", door_handle_pose.pose.position.y);
    RCLCPP_INFO(_LOGGER, "  z: %f", door_handle_pose.pose.position.z);

    mtc::Task task;
    task.stages()->setName("Open door task");
    task.loadRobotModel(shared_from_this());

    const auto& group_name = "mobile_base_arm";
    const auto& end_effector_name = "door_opening_end_effector";
    const auto& end_effector_parent_link = "robot/door_opening_mechanism_link_freely_rotating_hook";

    // Set task properties
    task.setProperty("group", group_name);
    task.setProperty("eef", end_effector_name);
    task.setProperty("ik_frame", end_effector_parent_link);

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

    // Current state
    {
      auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
      task.add(std::move(stage_state_current));
    }

    // Disable collision checking
    {
      auto modify_planning_scene_stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("Disable collision checking");
      modify_planning_scene_stage->allowCollisions("robot/right_wheel_link", "robot/right_wheel_tire_link", true);
      modify_planning_scene_stage->allowCollisions("robot/left_wheel_link", "robot/left_wheel_tire_link", true);
      task.add(std::move(modify_planning_scene_stage));
    }

    // Starting position
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("Starting position", sampling_planner);
      stage->setGroup(group_name);
      stage->setGoal("starting_position_middle");
      task.add(std::move(stage));
    }

    // Move above the door handle
    {
      // Connect the initial stage with the generated IK solution using the sampling planner.
      auto connector_stage = std::make_unique<moveit::task_constructor::stages::Connect>(
        "Connect", moveit::task_constructor::stages::Connect::GroupPlannerVector{{group_name, sampling_planner}});
      connector_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"eef", "group"});
      task.add(std::move(connector_stage));
    }

    // Generate a pose above the door handle and compute its IK
    {
      // Create a new pose based on door_handle_pose with a higher z coordinate
      geometry_msgs::msg::PoseStamped pose_above_door_handle = door_handle_pose;
      pose_above_door_handle.pose.position.z += DISTANCE_TO_PUSH_DOOR_HANDLE_DOWN;
      pose_above_door_handle.pose.position.x -= DOOR_DETECTION_OFFSET;

      // Generate a pose (this gets put in the IK wrapper below)
      auto generate_pose_above_door_handle_stage =
        std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate pose above door handle");
      generate_pose_above_door_handle_stage->setPose(pose_above_door_handle);
      generate_pose_above_door_handle_stage->setMonitoredStage(task.stages()->findChild("current"));

      // Compute IK
      auto ik_wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>(
        "generate pose above door handle IK", std::move(generate_pose_above_door_handle_stage));
      ik_wrapper->setMaxIKSolutions(3);
      ik_wrapper->setMinSolutionDistance(1.0);
      ik_wrapper->setTimeout(0.3);
      // ik_wrapper->setIKFrame(end_effector_parent_link);
      ik_wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"eef", "group"});
      ik_wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, {"target_pose"});
      task.add(std::move(ik_wrapper));
    }

    // Push door handle down
    {
      // Move the eef link forward along its z-axis by an amount within the given min-max range
      auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveRelative>("Push door handle down", cartesian_planner);
      stage->properties().set("link", end_effector_parent_link);   // link to perform IK for
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                            {"group"});   // inherit group from parent stage
      stage->setIKFrame(end_effector_parent_link);
      // stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = end_effector_parent_link;
      vec.vector.x = DISTANCE_TO_PUSH_DOOR_HANDLE_DOWN + Z_DOOR_DETECTION_OFFSET;
      stage->setDirection(vec);
      task.add(std::move(stage));
    }

    // // Connect the two stages
    // auto connector_stage_door_handle_push_down = std::make_unique<moveit::task_constructor::stages::Connect>(
    //   "Connect", moveit::task_constructor::stages::Connect::GroupPlannerVector{{group_name, sampling_planner}});
    // connector_stage_door_handle_push_down->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
    //                                                                       {"eef", "group"});
    // task.add(std::move(connector_stage_door_handle_push_down));

    // // Create a pose to push door handle down
    // geometry_msgs::msg::PoseStamped pose_below_door_handle = door_handle_pose;
    // pose_below_door_handle.pose.position.z -= (DISTANCE_TO_PUSH_DOOR_HANDLE_DOWN - Z_DOOR_DETECTION_OFFSET);
    // pose_below_door_handle.pose.position.x -= DOOR_DETECTION_OFFSET;

    // // Generate a pose (this gets put in the IK wrapper below)
    // auto generate_pose_below_door_handle_stage =
    //   std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate pose below door handle");
    // generate_pose_below_door_handle_stage->setPose(pose_below_door_handle);
    // generate_pose_below_door_handle_stage->setMonitoredStage(task.stages()->findChild("current"));

    // // Compute IK
    // auto ik_wrapper_below_door_handle = std::make_unique<moveit::task_constructor::stages::ComputeIK>(
    //   "generate pose below door handle IK", std::move(generate_pose_below_door_handle_stage));
    // ik_wrapper_below_door_handle->setMaxIKSolutions(3);
    // ik_wrapper_below_door_handle->setMinSolutionDistance(1.0);
    // ik_wrapper_below_door_handle->setTimeout(0.3);
    // ik_wrapper_below_door_handle->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
    //                                                              {"eef", "group"});
    // ik_wrapper_below_door_handle->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE,
    //                                                              {"target_pose"});
    // task.add(std::move(ik_wrapper_below_door_handle));

    // // Connect the two stages
    // auto connector_push_open_door = std::make_unique<moveit::task_constructor::stages::Connect>(
    //   "Connect", moveit::task_constructor::stages::Connect::GroupPlannerVector{{group_name, sampling_planner}});
    // connector_push_open_door->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"eef",
    // "group"});
    // // Create moveit_msgs/Constraints Message
    // // auto constraints = std::make_shared<moveit_msgs::msg::Constraints>();
    // // constraints->name = "push_open_door";
    // // connector_push_open_door->setPathConstraints(*(constraints.value()));
    // task.add(std::move(connector_push_open_door));

    // // Create a pose to push door open a little bit
    // geometry_msgs::msg::PoseStamped pose_push_open_door = door_handle_pose;
    // pose_push_open_door.pose.position.x -= (FORWARD_DISTANCE_TO_PUSH_DOOR_OUT_OF_LATCH + DOOR_DETECTION_OFFSET);
    // pose_push_open_door.pose.position.z -= (DISTANCE_TO_PUSH_DOOR_HANDLE_DOWN - Z_DOOR_DETECTION_OFFSET);

    // // Generate a pose (this gets put in the IK wrapper below)
    // auto generate_pose_push_open_door_stage =
    //   std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate pose push open door");
    // generate_pose_push_open_door_stage->setPose(pose_push_open_door);
    // generate_pose_push_open_door_stage->setMonitoredStage(task.stages()->findChild("current"));

    // // Compute IK
    // auto ik_wrapper_push_open_door = std::make_unique<moveit::task_constructor::stages::ComputeIK>(
    //   "generate pose push open door IK", std::move(generate_pose_push_open_door_stage));
    // ik_wrapper_push_open_door->setMaxIKSolutions(3);
    // ik_wrapper_push_open_door->setMinSolutionDistance(1.0);
    // ik_wrapper_push_open_door->setTimeout(0.3);
    // ik_wrapper_push_open_door->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
    //                                                           {"eef", "group"});
    // ik_wrapper_push_open_door->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE,
    //                                                           {"target_pose"});
    // task.add(std::move(ik_wrapper_push_open_door));

    // // Connect the two stages
    // auto connector_stage_move_gripper_above_handle = std::make_unique<moveit::task_constructor::stages::Connect>(
    //   "Connect", moveit::task_constructor::stages::Connect::GroupPlannerVector{{group_name, sampling_planner}});
    // connector_stage_move_gripper_above_handle->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
    //                                                                           {"eef", "group"});
    // task.add(std::move(connector_stage_move_gripper_above_handle));

    // // Create a pose to move the door handle above the door handle again
    // geometry_msgs::msg::PoseStamped pose_above_door_handle_again = door_handle_pose;
    // pose_above_door_handle_again.pose.position.x -=
    //   (FORWARD_DISTANCE_TO_PUSH_DOOR_OUT_OF_LATCH + DOOR_DETECTION_OFFSET);
    // pose_above_door_handle_again.pose.position.z += DISTANCE_TO_PUSH_DOOR_HANDLE_DOWN;

    // // Generate a pose (this gets put in the IK wrapper below)
    // auto generate_pose_above_door_handle_again_stage =
    //   std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate pose above door handle again");
    // generate_pose_above_door_handle_again_stage->setPose(pose_above_door_handle_again);
    // generate_pose_above_door_handle_again_stage->setMonitoredStage(task.stages()->findChild("current"));

    // // Compute IK
    // auto ik_wrapper_above_door_handle_again = std::make_unique<moveit::task_constructor::stages::ComputeIK>(
    //   "generate pose above door handle again IK", std::move(generate_pose_above_door_handle_again_stage));
    // ik_wrapper_above_door_handle_again->setMaxIKSolutions(3);
    // ik_wrapper_above_door_handle_again->setMinSolutionDistance(1.0);
    // ik_wrapper_above_door_handle_again->setTimeout(0.3);
    // ik_wrapper_above_door_handle_again->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
    //                                                                    {"eef", "group"});
    // ik_wrapper_above_door_handle_again->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE,
    //                                                                    {"target_pose"});
    // task.add(std::move(ik_wrapper_above_door_handle_again));

    // // Connect the two stages
    // auto connector_stage_push_door_open = std::make_unique<moveit::task_constructor::stages::Connect>(
    //   "Connect", moveit::task_constructor::stages::Connect::GroupPlannerVector{{group_name, sampling_planner}});
    // connector_stage_push_door_open->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
    //                                                                {"eef", "group"});
    // task.add(std::move(connector_stage_push_door_open));

    // // Create a pose to push door open
    // geometry_msgs::msg::PoseStamped pose_push_door_open = door_handle_pose;
    // pose_push_door_open.pose.position.x -= (FORWARD_DISTANCE_TO_PUSH_DOOR_OPEN + DOOR_DETECTION_OFFSET);
    // pose_push_door_open.pose.position.z += DISTANCE_TO_PUSH_DOOR_HANDLE_DOWN;
    // pose_push_door_open.pose.position.y += DISTANCE_TO_OTHER_SIDE_OF_DOOR;

    // // Generate a pose (this gets put in the IK wrapper below)
    // auto generate_pose_push_door_open_stage =
    //   std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate pose push door open");
    // generate_pose_push_door_open_stage->setPose(pose_push_door_open);
    // generate_pose_push_door_open_stage->setMonitoredStage(task.stages()->findChild("current"));

    // // Compute IK
    // auto ik_wrapper_push_door_open = std::make_unique<moveit::task_constructor::stages::ComputeIK>(
    //   "generate pose push door open IK", std::move(generate_pose_push_door_open_stage));
    // ik_wrapper_push_door_open->setMaxIKSolutions(3);
    // ik_wrapper_push_door_open->setMinSolutionDistance(1.0);
    // ik_wrapper_push_door_open->setTimeout(0.3);
    // ik_wrapper_push_door_open->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
    //                                                           {"eef", "group"});
    // ik_wrapper_push_door_open->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE,
    //                                                           {"target_pose"});
    // task.add(std::move(ik_wrapper_push_door_open));

    return task;
  }

}   // namespace door_opening_mechanism_mtc
