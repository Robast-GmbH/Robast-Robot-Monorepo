#include "door_opening_mechanism_mtc/door_opening_mechanism_mtc.hpp"

namespace door_opening_mechanism_mtc
{

  DoorMechanismMtc::DoorMechanismMtc() : Node("door_mechanism_mtc")
  {
    handle_node_parameter();
    create_subscriptions();
  }

  void DoorMechanismMtc::handle_node_parameter()
  {
    declare_parameter("moveit2_planning_group_name", _DEFAULT_PLANNING_GROUP_NAME);

    _planning_group_name = get_parameter("moveit2_planning_group_name").as_string();
  }

  void DoorMechanismMtc::create_subscriptions()
  {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    _door_handle_position_subscription = create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
      _DOOR_HANDLE_POSITION_TOPIC,
      qos,
      std::bind(&DoorMechanismMtc::door_handle_position_callback, this, std::placeholders::_1));
  }

  geometry_msgs::msg::PoseStamped DoorMechanismMtc::convert_pose_to_target_reference_frame(
    const geometry_msgs::msg::PoseStamped pose_in_source_frame, const std::string target_frame)
  {
    tf2_ros::Buffer tf_buffer(get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer, shared_from_this());

    try
    {
      // Wait for the transform from frame A to frame B to become available
      tf_buffer.canTransform(
        target_frame, pose_in_source_frame.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));

      // Convert the position from frame A to frame B
      geometry_msgs::msg::PoseStamped pose_in_target_frame = tf_buffer.transform(pose_in_source_frame, target_frame);

      // Print the transformed position
      RCLCPP_INFO(_LOGGER, "Position in target frame %s:", target_frame.c_str());
      RCLCPP_INFO(_LOGGER, "  x: %f", pose_in_target_frame.pose.position.x);
      RCLCPP_INFO(_LOGGER, "  y: %f", pose_in_target_frame.pose.position.y);
      RCLCPP_INFO(_LOGGER, "  z: %f", pose_in_target_frame.pose.position.z);

      return pose_in_target_frame;   // TODO: return this or better return a shared_ptr?
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(_LOGGER, "Transform from frame A to frame B not found: %s", ex.what());
    }
  }

  void DoorMechanismMtc::door_handle_position_callback(const depthai_ros_msgs::msg::SpatialDetectionArray& msg)
  {
    RCLCPP_INFO(_LOGGER,
                "I heard from the %s topic %i detections for the frame_id %s",
                _DOOR_HANDLE_POSITION_TOPIC.c_str(),
                msg.detections.size(),
                msg.header.frame_id.c_str());

    // Very important: We spin up the moveit interaction in new thread, otherwise
    // the current state monitor won't get any information about the robot's state.
    std::thread{std::bind(&DoorMechanismMtc::open_door_in_simulation, this, std::placeholders::_1),
                std::make_shared<depthai_ros_msgs::msg::SpatialDetectionArray>(msg)}
      .detach();
  }

  void DoorMechanismMtc::open_door_in_simulation(
    const std::shared_ptr<depthai_ros_msgs::msg::SpatialDetectionArray> door_handle_poses)
  {
    for (uint8_t i = 0; i < door_handle_poses->detections.size(); i++)
    {
      RCLCPP_INFO(_LOGGER, "Target position for detection %i:", i);
      RCLCPP_INFO(_LOGGER, "   x = %f", door_handle_poses->detections[i].position.x);
      RCLCPP_INFO(_LOGGER, "   y = %f", door_handle_poses->detections[i].position.y);
      RCLCPP_INFO(_LOGGER, "   z = %f", door_handle_poses->detections[i].position.z);
    }

    geometry_msgs::msg::PoseStamped pose_in_source_frame;
    pose_in_source_frame.header = door_handle_poses->header;
    pose_in_source_frame.pose.position = door_handle_poses->detections[0].position;
    geometry_msgs::msg::PoseStamped target_pose =
      convert_pose_to_target_reference_frame(pose_in_source_frame, "base_footprint");

    do_task(target_pose);
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

  void DoorMechanismMtc::do_task(geometry_msgs::msg::PoseStamped target_pose)
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
    if (!_task.plan(5 /* max_solutions */))
    {
      RCLCPP_ERROR(_LOGGER, "Task planning failed");
      return;
    }
    RCLCPP_INFO(_LOGGER, "Publishing task solutions!");
    _task.introspection().publishSolution(*_task.solutions().front());

    RCLCPP_INFO(_LOGGER, "Executing task!");
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

    double roll = 1.57079632679;
    double pitch = 1.57079632679;
    double yaw = 0.0;
    // Create a Quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    target_pose.pose.orientation.x = quaternion.x();
    target_pose.pose.orientation.y = quaternion.y();
    target_pose.pose.orientation.z = quaternion.z();
    target_pose.pose.orientation.w = quaternion.w();

    mtc::Task task;
    task.stages()->setName("approach door handle task");
    task.loadRobotModel(shared_from_this());

    // TODO@Jacob: Executing a task works only for the door_opening_mechanism for now.
    // Since we are using the joint_trajectory_controller from ros2_control now, we have to find a way how we
    // incorporate the mobile base into this properly
    const auto& group_name = "door_opening_mechanism";   // mobile_base_arm or door_opening_mechanism
    const auto& group_name_arm = "door_opening_mechanism";
    const auto& end_effector_name = "door_opening_end_effector";
    const auto& end_effector_parent_link = "door_opening_mechanism_link_freely_rotating_hook";

    const auto& hand_group_name = "hand";
    const auto& hand_frame = "hook_base_link";

    // Set task properties
    task.setProperty("group", group_name);
    task.setProperty("eef", end_effector_name);
    task.setProperty("ik_frame", end_effector_parent_link);

    mtc::Stage* current_state_ptr = nullptr;   // Forward current_state on to grasp pose generator
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
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

    // Connect the initial stage with the generated IK solution using the sampling planner.
    auto connector_stage = std::make_unique<moveit::task_constructor::stages::Connect>(
      "Connect", moveit::task_constructor::stages::Connect::GroupPlannerVector{{group_name, sampling_planner}});
    connector_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"eef", "group"});
    task.add(std::move(connector_stage));

    // Generate a pose (this gets put in the IK wrapper below)
    auto generate_pose_stage = std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate pose");
    generate_pose_stage->setPose(target_pose);
    generate_pose_stage->setMonitoredStage(task.stages()->findChild("current"));

    // Compute IK
    auto ik_wrapper =
      std::make_unique<moveit::task_constructor::stages::ComputeIK>("generate pose IK", std::move(generate_pose_stage));
    ik_wrapper->setMaxIKSolutions(5);
    ik_wrapper->setMinSolutionDistance(1.0);
    ik_wrapper->setTimeout(1.0);
    // ik_wrapper->setIKFrame(end_effector_parent_link);
    ik_wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"eef", "group"});
    ik_wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, {"target_pose"});
    task.add(std::move(ik_wrapper));

    // TODO@Jacob: After moving to iron and gz_ros2_control this makes problems. Fix this when dealing with mtc again
    // auto stage = std::make_unique<mtc::stages::MoveTo>("Starting position", interpolation_planner);
    // stage->setGroup(group_name);
    // stage->setGoal("starting_position");
    // task.add(std::move(stage));

    return task;
  }

}   // namespace door_opening_mechanism_mtc
