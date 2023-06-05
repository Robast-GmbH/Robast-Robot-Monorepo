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

    this->door_handle_position_subscription_ = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
        door_handle_position_topic_,
        qos,
        std::bind(&DoorMechanismSimulation::door_handle_position_callback, this, std::placeholders::_1));
  }

  std::shared_ptr<rclcpp::Node> DoorMechanismSimulation::get_shared_pointer_of_node()
  {
    return std::enable_shared_from_this<rclcpp::Node>::shared_from_this();
  }

  geometry_msgs::msg::PoseStamped DoorMechanismSimulation::convert_pose_to_target_reference_frame(
      const geometry_msgs::msg::PoseStamped pose_in_source_frame, const std::string target_frame)
  {
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer, this->get_shared_pointer_of_node());

    try
    {
      // Wait for the transform from frame A to frame B to become available
      tf_buffer.canTransform(
          target_frame, pose_in_source_frame.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));

      // Convert the position from frame A to frame B
      geometry_msgs::msg::PoseStamped pose_in_target_frame = tf_buffer.transform(pose_in_source_frame, target_frame);

      // Print the transformed position
      RCLCPP_INFO(this->get_logger(), "Position in target frame %s:", target_frame.c_str());
      RCLCPP_INFO(this->get_logger(), "  x: %f", pose_in_target_frame.pose.position.x);
      RCLCPP_INFO(this->get_logger(), "  y: %f", pose_in_target_frame.pose.position.y);
      RCLCPP_INFO(this->get_logger(), "  z: %f", pose_in_target_frame.pose.position.z);

      return pose_in_target_frame;   // TODO: return this or better return a shared_ptr?
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform from frame A to frame B not found: %s", ex.what());
    }
  }

  void DoorMechanismSimulation::move_robot_in_simulation_to_target_pose(geometry_msgs::msg::PoseStamped target_pose)
  {
    const std::string PLANNING_GROUP = this->moveit2_planning_group_name_;

    // The
    // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
    // class can be easily set up using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(this->get_shared_pointer_of_node(), PLANNING_GROUP);

    // We will use the
    // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(
        this->get_shared_pointer_of_node(), "base_footprint", "move_group_tutorial", move_group.getRobotModel());

    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script */
    /* via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start planning!");

    joint_model_group->printGroupInfo();

    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(this->get_logger(),
                "Current pose: x = %f, y = %f, z = %f, w = %f, orientation_x = %f, orientation_y = %f, orientation_z = "
                "%f, frame_id = %s",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z,
                current_pose.pose.orientation.w,
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.header.frame_id.c_str());

    target_pose.pose.orientation = current_pose.pose.orientation;

    RCLCPP_INFO(this->get_logger(),
                "Target pose: x = %f, y = %f, z = %f, w = %f, orientation_x = %f, orientation_y = %f, orientation_z = "
                "%f, frame_id = %s",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.pose.orientation.w,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.header.frame_id.c_str());

    move_group.setGoalTolerance(0.00001);
    // move_group.setPlannerId("TRRTkConfigDefault");
    move_group.setPlanningTime(5);
    move_group.setNumPlanningAttempts(1);
    move_group.setPoseTarget(target_pose);

    RCLCPP_INFO(this->get_logger(), "move_group.getPlanningTime(): %f", move_group.getPlanningTime());

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // We can also visualize the plan as a line with markers in RViz.
    RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose.pose, "pose1");
    visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    if (success)
    {
      visual_tools.prompt(
          "Planning succeeded! Press 'next' in the RvizVisualToolsGui window to start executing the plan!");
      move_group.execute(my_plan);
    }

    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to delete all markers");

    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
  }

  void DoorMechanismSimulation::open_door_in_simulation(
      const std::shared_ptr<depthai_ros_msgs::msg::SpatialDetectionArray> door_handle_poses)
  {
    for (uint8_t i = 0; i < door_handle_poses->detections.size(); i++)
    {
      RCLCPP_INFO(this->get_logger(), "Target position for detection %i:", i);
      RCLCPP_INFO(this->get_logger(), "   x = %f", door_handle_poses->detections[i].position.x);
      RCLCPP_INFO(this->get_logger(), "   y = %f", door_handle_poses->detections[i].position.y);
      RCLCPP_INFO(this->get_logger(), "   z = %f", door_handle_poses->detections[i].position.z);
    }

    geometry_msgs::msg::PoseStamped pose_in_source_frame;
    pose_in_source_frame.header = door_handle_poses->header;
    pose_in_source_frame.pose.position = door_handle_poses->detections[0].position;
    geometry_msgs::msg::PoseStamped target_pose = convert_pose_to_target_reference_frame(pose_in_source_frame, "odom");

    this->move_robot_in_simulation_to_target_pose(target_pose);
  }

  void DoorMechanismSimulation::door_handle_position_callback(const depthai_ros_msgs::msg::SpatialDetectionArray& msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "I heard from the %s topic %i detections for the frame_id %s",
                door_handle_position_topic_.c_str(),
                msg.detections.size(),
                msg.header.frame_id);

    // Very important: We spin up the moveit interaction in new thread, otherwise
    // the current state monitor won't get any information about the robot's state.
    std::thread{std::bind(&DoorMechanismSimulation::open_door_in_simulation, this, std::placeholders::_1),
                std::make_shared<depthai_ros_msgs::msg::SpatialDetectionArray>(msg)}
        .detach();
  }
}   // namespace door_opening_mechanism_simulation