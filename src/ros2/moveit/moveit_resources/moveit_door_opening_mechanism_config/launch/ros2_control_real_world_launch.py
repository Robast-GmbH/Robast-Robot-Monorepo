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



def shutdown_dryve_d1():
    print("Shutting down dryve d1 motor controller now!")
    os.system("ros2 run dryve_d1_bridge shutdown_dryve_d1")


def generate_moveit_config(context, *args, **settings):
    prefix = LaunchConfiguration("prefix").perform(context)
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type").perform(context)
    ros2_control_hardware_type_positon_joint = LaunchConfiguration("ros2_control_hardware_type_positon_joint").perform(context)
    model_door_opening_mechanism = LaunchConfiguration("model_door_opening_mechanism").perform(context)
    model_module_cage = LaunchConfiguration("model_module_cage").perform(context)
    model_sensors = LaunchConfiguration("model_sensors").perform(context)
    position_joint_type = LaunchConfiguration("position_joint_type").perform(context)
    robot_description_path = LaunchConfiguration("robot_description_path").perform(context)

    robot_description_semantic_file_path = LaunchConfiguration("robot_description_semantic_file_path").perform(context)
    trajectory_execution_file_path = LaunchConfiguration("trajectory_execution_file_path").perform(context)
    sensors_3d_file_path = LaunchConfiguration("sensors_3d_file_path").perform(context)
    joint_limits_file_path = LaunchConfiguration("joint_limits_file_path").perform(context)

    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "humble":
        planning_pipelines = ["ompl_humble"]
    elif ros_distro == "iron":
        planning_pipelines = ["ompl_iron"]
    else:
        raise Exception("Unknown ROS distro: " + ros_distro)

    launch_arguments_moveit_config = {
        "ros2_control_hardware_type": ros2_control_hardware_type,
        "position_joint_type": ros2_control_hardware_type_positon_joint,
        "model_door_opening_mechanism": model_door_opening_mechanism,
        "model_module_cage": model_module_cage,
        "model_sensors": model_sensors,
        "ros2_control_hardware_type_positon_joint": ros2_control_hardware_type_positon_joint,
        "prefix": PREFIX,
    }
        
    moveit_config = (
        MoveItConfigsBuilder(
            "rb_theron",
            package_name="moveit_door_opening_mechanism_config",
        )
        .robot_description(
            file_path=robot_description_path, mappings=launch_arguments_moveit_config
        )
        .robot_description_semantic(file_path=robot_description_semantic_file_path)
        .trajectory_execution(file_path=trajectory_execution_file_path)
        .planning_pipelines(pipelines=planning_pipelines)
        .sensors_3d(file_path=sensors_3d_file_path)
        .joint_limits(file_path=joint_limits_file_path)
        .to_moveit_configs()
    )


def generate_launch_description():

    declare_prefix_cmd = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description="The prefix to be used for the robot description",
    )

    declare_ros2_control_hardware_type_cmd = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="dryve_d1",
        description="The hardware type to use for ros2 control",
    )

    declare_ros2_control_hardware_type_positon_joint_cmd = DeclareLaunchArgument(
        "ros2_control_hardware_type_positon_joint",
        default_value="real_life",
        description="The hardware type to use for ros2 control for position joint",
    )

    declare_robot_description_path_cmd = DeclareLaunchArgument(
        "robot_description_path",
        default_value="",
        description="The path to the robot description file",
    )

    declare_position_joint_type_cmd = DeclareLaunchArgument(
        "position_joint_type",
        default_value="prismatic",
        description="The type of the position joint",
    )

    declare_model_door_opening_mechanism_cmd = DeclareLaunchArgument(
        "model_door_opening_mechanism",
        default_value="true",
        description="Whether to include the door opening mechanism in the model",
    )

    declare_model_module_cage_cmd = DeclareLaunchArgument(
        "model_module_cage",
        default_value="true",
        description="Whether to include the module cage in the model",
    )

    declare_model_sensors_cmd = DeclareLaunchArgument(
        "model_sensors",
        default_value="true",
        description="Whether to include the sensors in the model",
    )

    declare_robot_description_semantic_file_path_cmd = DeclareLaunchArgument(
        "robot_description_semantic_file_path",
        default_value="",
        description="The path to the robot description semantic file",
    )

    declare_trajectory_execution_file_path_cmd = DeclareLaunchArgument(
        "trajectory_execution_file_path",
        default_value="",
        description="The path to the trajectory execution file",
    )

    declare_sensors_3d_file_path_cmd = DeclareLaunchArgument(
        "sensors_3d_file_path",
        default_value="",
        description="The path to the sensors 3d file",
    )

    declare_joint_limits_file_path_cmd = DeclareLaunchArgument(
        "joint_limits_file_path",
        default_value="",
        description="The path to the joint limits file",
    )

    moveit_config = OpaqueFunction(function=generate_moveit_config)

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

    ld = LaunchDescription()

    ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_ros2_control_hardware_type_cmd)
    ld.add_action(declare_ros2_control_hardware_type_positon_joint_cmd)
    ld.add_action(declare_robot_description_path_cmd)
    ld.add_action(declare_position_joint_type_cmd)
    ld.add_action(declare_model_door_opening_mechanism_cmd)
    ld.add_action(declare_model_module_cage_cmd)
    ld.add_action(declare_model_sensors_cmd)
    ld.add_action(declare_robot_description_semantic_file_path_cmd)
    ld.add_action(declare_trajectory_execution_file_path_cmd)
    ld.add_action(declare_sensors_3d_file_path_cmd)
    ld.add_action(declare_joint_limits_file_path_cmd)

    ld.add_action(rosout_listener_trigger)
    ld.add_action(move_group_cmd)

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
