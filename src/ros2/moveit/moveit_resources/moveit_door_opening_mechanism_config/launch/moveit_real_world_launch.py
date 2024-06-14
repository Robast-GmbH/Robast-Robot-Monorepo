import os
import atexit
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

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

PREFIX = "robot/"


def shutdown_dryve_d1():
    print("Shutting down dryve d1 motor controller now!")
    os.system("ros2 run dryve_d1_bridge shutdown_dryve_d1")


def launch_robot_state_publisher():
    robot_xml = xacro.process_file(
        os.path.join(
            get_package_share_directory("rb_theron_description"),
            "robots",
            "rb_theron_arm.urdf.xacro",
        ),
        mappings={
            "ros2_control_hardware_type": "dryve_d1",
            "ros2_control_hardware_type_positon_joint": "real_life",
            "ros_distro": os.environ["ROS_DISTRO"],
            "prefix": PREFIX,
        },
    ).toxml()

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_xml},
        ],
        output="screen",
    )

    return start_robot_state_publisher_cmd


def generate_launch_description():
    launch_arguments = {
        "ros2_control_hardware_type": "dryve_d1",
        "model_position_joint": "prismatic",
        "model_door_opening_mechanism": "true",
        "model_module_cage": "false",
        "model_sensors": "false",
        "ros2_control_hardware_type_positon_joint": "real_life",
        "prefix": PREFIX,
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
        .robot_description_semantic(file_path="config/rb_theron_real_world.srdf")
        .trajectory_execution(file_path="config/moveit_controllers_real_world.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .sensors_3d(file_path="config/sensors_3d_real_world.yaml")
        .joint_limits(file_path="config/joint_limits_real_world.yaml")
        .to_moveit_configs()
    )

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

    # Start the actual move_group node/action server
    move_group_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # ros2_control
    ros2_controllers_path_arm = os.path.join(
        get_package_share_directory("moveit_door_opening_mechanism_config"),
        "config",
        "ros2_controllers_real_world_arm.yaml",
    )
    CONTROLLER_MANAGER_NAME = "/controller_manager"
    ros2_controller_manager_arm_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path_arm,
        ],
        output="both",
    )

    # This node listens to the rosout topic and exits if a certain trigger_message is found in the
    # rosout message. This is usefull in combination with the OnProcessExit event handler.
    # Here we need this to wait until homing of the arm is finished before we load the controllers.
    # By some testing I found out that the homing is finished when the message "update rate is xHz"
    # is published by the controller_manager.
    # TODO@all: This is not a perfect solution, maybe someone finds something better at some point
    TRIGGER_MESSAGE = "update rate is"
    node_name = "controller_manager"
    rosout_listener_trigger = Node(
        package="launch_manager",
        executable="rosout_listener",
        name="rosout_listener_trigger",
        output="log",
        parameters=[
            {"trigger_message": TRIGGER_MESSAGE, "node_name": node_name},
        ],
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

    ld.add_action(declare_debug_cmd)

    ld.add_action(rosout_listener_trigger)
    ld.add_action(static_virtual_joint_cmd)
    ld.add_action(move_group_cmd)
    ld.add_action(launch_robot_state_publisher())

    # Make sure that the rosout_listener_trigger is started before the arm controller manager is
    # started because the rosout_listener_trigger should listen to the content of the arm
    # controller manager
    ld.add_action(TimerAction(period=1.0, actions=[ros2_controller_manager_arm_cmd]))

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

    # This would be probably much nice to use RegisterEventHandler to trigger the shutdown, but
    # unfortunately we did not get this running.
    # Therefore we shutdown the dryve d1 with the "atexit" library.
    atexit.register(shutdown_dryve_d1)

    return ld
