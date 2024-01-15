import os
import atexit

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

"""
    Launches a self contained demo

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * ros2_control_node + controller spawners
"""

def shutdown_dryve_d1():
    print("Shutting down dryve d1 motor controller now!")
    os.system('ros2 run dryve_d1_bridge shutdown_dryve_d1')

def generate_launch_description():

    launch_arguments = {
        "ros2_control_hardware_type": "dryve_d1",
        "ros2_control_hardware_type_positon_joint": "real_life",
        "model_position_joint": "true",
    }

    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "humble":
        planning_pipelines = ["ompl_humble"]
    else:
        planning_pipelines = ["ompl_iron"]

    moveit_config = (
        MoveItConfigsBuilder("rb_theron", package_name="moveit_door_opening_mechanism_rotating_arm_config")
        .robot_description_semantic(file_path="config/rb_theron_arm.srdf")
        .robot_description(file_path="config/rb_theron_arm.urdf.xacro", mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .to_moveit_configs()
    )
    namespace_arm = "arm"

    ld = LaunchDescription()

    declare_debug_cmd = DeclareBooleanLaunchArg(
        "debug",
        default_value=False,
        description="By default, we are not in debug mode",
    )

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    static_virtual_joint_cmd = generate_static_virtual_joint_tfs_launch(moveit_config)

    # Load ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # For our door opening mechanism the current idea is to have two separate tf trees.
    # (1) The "normal" tf tree used for navigation
    # (2) The "arm" tf tree used for the door opening mechanism
    # The reason we need to separate this is that we have put a position joint between the base_footprint and the base_link
    # in order to plan for the motion of the base.
    # However, this joint is not part of the real robot and therefore we need to remove it from the tf tree used for the arm.
    # This is done by remapping the tf topics.
    remappings_move_group = [
        ("/tf", "/tf_dump"),
        ("/tf_static", "/tf_static_dump"),
        ("/" + namespace_arm + "/tf", "/tf"),
        ("/" + namespace_arm + "/tf_static", "/tf_static"),
    ]    

    # Start the actual move_group node/action server
    move_group_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace=namespace_arm,
        remappings=remappings_move_group,
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    rviz_config_file = (
        get_package_share_directory("moveit_door_opening_mechanism_rotating_arm_config") + "/config/moveit.rviz"
    )
    rviz2_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    remappings_state_publisher = [
        ("/tf", "/" + namespace_arm + "/tf"),
        ("/tf_static", "/" + namespace_arm + "/tf_static"),
    ]    

    # State Publisher
    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        namespace=namespace_arm,
        remappings=remappings_state_publisher,
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control
    ros2_controllers_path_arm = os.path.join(
        get_package_share_directory("moveit_door_opening_mechanism_rotating_arm_config"),
        "config",
        "ros2_controllers_real_world_arm.yaml",
    )
    controller_manager_name = "/" + namespace_arm + "/controller_manager"
    remappings_controller_manager_arm = [
        ("/" + namespace_arm + "/robot/robotnik_base_control/cmd_vel", "/robot/robotnik_base_control/cmd_vel"),
    ]
    ros2_controller_manager_arm_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace_arm,
        remappings=remappings_controller_manager_arm,
        parameters=[moveit_config.robot_description, ros2_controllers_path_arm,],
        output="both",
    )
    
    load_mobile_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'mobile_base_controller_cmd_vel', '-c', controller_manager_name],
        output='screen'
    )
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster', '-c', controller_manager_name],
        output='screen'
    )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller', '-c', controller_manager_name],
        output='screen'
    )

    ld.add_action(declare_debug_cmd)

    ld.add_action(static_virtual_joint_cmd)
    ld.add_action(rviz2_cmd)
    ld.add_action(move_group_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(ros2_controller_manager_arm_cmd)

    # spawning ros2_control controller for arm
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ros2_controller_manager_arm_cmd,
                on_start=[load_joint_state_broadcaster],
            )
        ))
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_mobile_base_controller],
            )
        ))
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_mobile_base_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ))

    # This would be probably much nice to use RegisterEventHandler to trigger the shutdown, but unfortunately we
    # did not get this running. Therefore we shutdown the dryve d1 with the "atexit" library.
    atexit.register(shutdown_dryve_d1)

    return ld