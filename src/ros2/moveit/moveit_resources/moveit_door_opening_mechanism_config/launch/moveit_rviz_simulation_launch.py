import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch

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


def generate_launch_description():
    launch_arguments = {
        "ros2_control_hardware_type": "mock_components",
        "ros2_control_hardware_type_positon_joint": "real_life",
        "model_position_joint": "true",
    }

    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "humble":
        planning_pipelines = ["ompl_humble"]
    elif ros_distro == "iron":
        planning_pipelines = ["ompl_iron"]
    else:
        raise Exception("Unknown ROS distro: " + ros_distro)

    moveit_config = (
        MoveItConfigsBuilder(
            "rb_theron",
            package_name="moveit_door_opening_mechanism_config",
        )
        .robot_description(
            file_path="config/rb_theron.urdf.xacro", mappings=launch_arguments
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .to_moveit_configs()
    )
    NAMESPACE_ARM = "arm"

    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="whether to use sim time or not",
    )

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

    JOINT_STATES_TOPIC = "/joint_states"
    remappings = [
        ("/" + NAMESPACE_ARM + JOINT_STATES_TOPIC, JOINT_STATES_TOPIC),
    ]
    move_group_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace=NAMESPACE_ARM,
        remappings=remappings,
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    rviz_config_file = (
        get_package_share_directory("moveit_door_opening_mechanism_config")
        + "/config/moveit.rviz"
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

    # State Publisher
    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    ros2_controllers_path_base = os.path.join(
        get_package_share_directory("moveit_door_opening_mechanism_config"),
        "config",
        "ros2_controllers_simulation_base.yaml",
    )
    ros2_controllers_path_arm = os.path.join(
        get_package_share_directory("moveit_door_opening_mechanism_config"),
        "config",
        "ros2_controllers_simulation_arm.yaml",
    )

    # Please mind:
    # We could use only one controller manager for both the robot base and the arm.
    # But on our real robot we already have a controller manager running, that is activating the
    # diff_drive_base_controller.
    # This gives us two options:
    # 1. Change the controller manager configs on the real robot to also load the arm controllers
    # 2. Start a second controller manager for the arm
    # We decided to go with option 2, because it is less intrusive.
    # So to keep the simulation as close to the real robot as possible, we also start a second
    # controller manager here.

    # First controller manager for the robot base
    ros2_controller_manager_robot_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path_base,
            {"use_sim_time": use_sim_time},
        ],
        output="both",
    )
    load_diff_drive_base_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_base_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    # Second controller manager for the arm
    CONTROLLER_MANAGER_NAME = "/" + NAMESPACE_ARM + "/controller_manager"
    remappings_controller_manager_arm = [
        (
            "/" + NAMESPACE_ARM + "/diff_drive_base_controller/cmd_vel",
            "/diff_drive_base_controller/cmd_vel",
        ),
        ("/" + NAMESPACE_ARM + JOINT_STATES_TOPIC, JOINT_STATES_TOPIC),
    ]
    ros2_controller_manager_arm_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=NAMESPACE_ARM,
        remappings=remappings_controller_manager_arm,
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path_arm,
            {"use_sim_time": use_sim_time},
        ],
        output="both",
    )

    load_mobile_base_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "mobile_base_controller_cmd_vel",
            "-c",
            CONTROLLER_MANAGER_NAME,
        ],
        output="screen",
    )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
            "-c",
            CONTROLLER_MANAGER_NAME,
        ],
        output="screen",
    )
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
            "-c",
            CONTROLLER_MANAGER_NAME,
        ],
        output="screen",
    )

    # This node listens to the rosout topic and exits if a certain trigger_message is found in the
    # rosout message. This is usefull in combination with the OnProcessExit event handler.
    # Here we need this to wait until homing of the arm is finished before we load the controllers.
    # By some testing I found out that the homing is finished when the message "update rate is xHz"
    # is published by the controller_manager.
    # TODO@all: This is not a perfect solution, maybe someone finds something better at some point
    trigger_message = "update rate is"
    node_name = NAMESPACE_ARM + ".controller_manager"
    rosout_listener_trigger = Node(
        package="launch_manager",
        executable="rosout_listener",
        name="rosout_listener_trigger",
        output="log",
        parameters=[
            {"trigger_message": trigger_message, "node_name": node_name},
        ],
    )

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_debug_cmd)

    ld.add_action(rosout_listener_trigger)
    ld.add_action(static_virtual_joint_cmd)
    ld.add_action(rviz2_cmd)
    ld.add_action(move_group_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(ros2_controller_manager_robot_cmd)

    # Make sure that the rosout_listener_trigger is started before the arm controller manager is
    # started because the rosout_listener_trigger should listen to the content of the arm
    # controller manager
    ld.add_action(TimerAction(period=1.0, actions=[ros2_controller_manager_arm_cmd]))

    # spawning ros2_control controller for robot base
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rosout_listener_trigger,
                on_exit=[load_diff_drive_base_controller],
            )
        )
    )

    # spawning ros2_control controller for arm
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rosout_listener_trigger,
                on_exit=[load_joint_state_broadcaster],
            )
        )
    )
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_mobile_base_controller],
            )
        )
    )
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_mobile_base_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        )
    )

    return ld
