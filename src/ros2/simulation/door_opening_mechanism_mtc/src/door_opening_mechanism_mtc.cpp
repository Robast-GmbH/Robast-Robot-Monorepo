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
    geometry_msgs::msg::PoseStamped target_pose = convert_pose_to_target_reference_frame(pose_in_source_frame, "odom");

    do_task();

    // move_robot_in_simulation_to_target_pose(target_pose); //TODO: uncomment this
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

  void DoorMechanismMtc::do_task()
  {
    _task = create_task();

    try
    {
      _task.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(_LOGGER, e);
      return;
    }

    if (!_task.plan(5 /* max_solutions */))
    {
      RCLCPP_ERROR_STREAM(_LOGGER, "Task planning failed");
      return;
    }
    _task.introspection().publishSolution(*_task.solutions().front());

    auto result = _task.execute(*_task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(_LOGGER, "Task execution failed");
      return;
    }

    return;
  }

  mtc::Task DoorMechanismMtc::create_task()
  {
    RCLCPP_INFO(_LOGGER, "Creating task!");

    mtc::Task task;
    task.stages()->setName("approach door handle task");
    task.loadRobotModel(shared_from_this());

    const auto& mobile_base_group_name = "mobile_base_arm";
    const auto& manipulator_group_name = "door_opening_mechanism";
    const auto& hand_frame = "panda_hand";   // TODO: What is this for?

    // Set task properties
    task.setProperty("group", mobile_base_group_name);
    task.setProperty("eef", manipulator_group_name);
    task.setProperty("ik_frame", hand_frame);   // TODO: What is this for?

    mtc::Stage* current_state_ptr = nullptr;   // Forward current_state on to grasp pose generator
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(mobile_base_group_name);
    stage_open_hand->setGoal("test_state");
    task.add(std::move(stage_open_hand));

    return task;
  }

}   // namespace door_opening_mechanism_mtc
