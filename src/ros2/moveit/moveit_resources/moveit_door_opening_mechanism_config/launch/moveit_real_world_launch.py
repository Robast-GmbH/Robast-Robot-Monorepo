import os
import atexit

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


def shutdown_dryve_d1():
    print("Shutting down dryve d1 motor controller now!")
    os.system("ros2 run dryve_d1_bridge shutdown_dryve_d1")


def generate_launch_description():
    launch_arguments = {
        "ros2_control_hardware_type": "dryve_d1",
        "ros2_control_hardware_type_positon_joint": "real_life",
        "model_position_joint": "prismatic",
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
            file_path="config/rb_theron_arm.urdf.xacro", mappings=launch_arguments
        )
        .robot_description_semantic(file_path="config/rb_theron_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .sensors_3d(file_path="config/sensors_3d_real_world.yaml")
        .to_moveit_configs()
    )
    NAMESPACE_ARM = "arm"

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
    # The reason we need to separate this is that we have put a position joint between the
    # base_footprint and the base_link in order to plan for the motion of the base.
    # However, this joint is not part of the real robot and therefore we need to remove it from the
    # tf tree used for the arm. This is done by remapping the tf topics.
    TF_TOPIC = "/tf"
    TF_STATIC_TOPIC = "/tf_static"
    tf_remapping = [
        (TF_TOPIC, "/" + NAMESPACE_ARM + TF_TOPIC),
        (TF_STATIC_TOPIC, "/" + NAMESPACE_ARM + TF_STATIC_TOPIC),
    ]

    # Start the actual move_group node/action server
    move_group_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace=NAMESPACE_ARM,
        remappings=tf_remapping,
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # State Publisher
    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        namespace=NAMESPACE_ARM,
        remappings=tf_remapping,
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control
    ros2_controllers_path_arm = os.path.join(
        get_package_share_directory("moveit_door_opening_mechanism_config"),
        "config",
        "ros2_controllers_real_world_arm.yaml",
    )
    CONTROLLER_MANAGER_NAME = "/" + NAMESPACE_ARM + "/controller_manager"
    remappings_controller_manager_arm = [
        (
            "/" + NAMESPACE_ARM + "/robot/robotnik_base_control/cmd_vel",
            "/robot/robotnik_base_control/cmd_vel",
        ),
    ]
    ros2_controller_manager_arm_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=NAMESPACE_ARM,
        remappings=remappings_controller_manager_arm,
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
    node_name = NAMESPACE_ARM + ".controller_manager"
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
    ld.add_action(robot_state_publisher_cmd)

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
