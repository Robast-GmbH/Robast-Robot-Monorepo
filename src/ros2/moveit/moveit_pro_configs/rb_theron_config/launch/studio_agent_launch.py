# Copyright 2021 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from moveit_studio_utils_py.system_config import (
    get_config_folder,
    SystemConfigParser,
)


def launch_setup(context, *args, **kwargs):
    system_config_parser = SystemConfigParser()

    # The Studio Agent launch file writes the system config YAML to file so it's available for debugging purposes.
    system_config_parser.write_complete_config_yaml_to_file()
    # Store robot description in a file for REST API
    if not (system_config_parser.write_robot_description_to_file()):
        print(
            "\nFailed to save robot description to file. Without this, the REST API won't work. Shutting down the agent.\n"
        )
        exit()

    # Import the robot configuration and generate config folder.
    system_config = system_config_parser.get_system_config()

    # Split the system dict into separate dicts, validating along the way.
    robot_config_name = system_config_parser.get_site_config_package()
    if robot_config_name is None:
        raise Exception(
            'ERROR: Your configuration must specify a "site_config_package" name.'
        )

    hardware_setting = system_config_parser.get_hardware_config()
    is_simulated = hardware_setting.get("simulated", False)
    moveit_setting = system_config_parser.get_moveit_configs()
    ros_configs = system_config_parser.get_ros_global_configs()
    use_sim_time = ros_configs.get("use_sim_time", False)

    print("")
    print("********************************************************************")
    print("*")
    print("*     Starting Robot: ", robot_config_name)
    print("*          Simulated: ", str(is_simulated))
    print("*")
    print("********************************************************************")
    print("")

    # Specify launch arguments
    log_level_config = LaunchConfiguration("log_level")
    objective_library_directories_config = LaunchConfiguration(
        "objective_library_directories"
    )
    is_test_config = LaunchConfiguration("is_test")

    # Define nodes to launch
    nodes_to_launch = []

    robot_description = {"robot_description": system_config_parser.get_processed_urdf()}
    robot_description_semantic = {
        "robot_description_semantic": system_config_parser.get_processed_srdf()
    }

    move_group_parameters = [
        moveit_setting["robot_description_kinematics"],
        moveit_setting["robot_description_planning"],
        moveit_setting["planning_pipeline"],
        moveit_setting["trajectory_execution"],
        moveit_setting["moveit_controller_manager"],
        moveit_setting["moveit_simple_controller_manager"],
        moveit_setting["planning_scene_monitor_parameters"],
        moveit_setting["move_group_capabilities"],
        moveit_setting["sensors_3d_parameters"],
        ros_configs,
        robot_description,
    ]

    # Start the actual move_group node/action server
    nodes_to_launch.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="both",
            parameters=move_group_parameters,
            arguments=["--ros-args", "--log-level", log_level_config],
        )
    )

    nodes_to_launch.append(
        Node(
            package="moveit_studio_agent",
            executable="parameter_manager_node",
            name="parameter_manager_node",
            output="both",
            parameters=[ros_configs],
            arguments=["--ros-args", "--log-level", log_level_config],
        )
    )

    nodes_to_launch.append(
        Node(
            package="moveit_studio_agent",
            executable="waypoint_manager_node",
            name="waypoint_manager_node",
            output="both",
            parameters=[
                {
                    "serialization_file": os.path.join(
                        get_config_folder(), "waypoints.yaml"
                    )
                },
                moveit_setting["joint_group_name"],
                ros_configs,
                robot_description,
            ],
            arguments=["--ros-args", "--log-level", log_level_config],
        )
    )

    nodes_to_launch.append(
        Node(
            package="moveit_studio_agent",
            executable="move_joint_resampler_node",
            name="move_joint_resampler_node",
            parameters=[
                {
                    "input_reset_rate": 4.0,
                    "input_stop_rate": 2.0,
                    "use_sim_time": use_sim_time,
                },
            ],
            arguments=["--ros-args", "--log-level", log_level_config],
        )
    )

    nodes_to_launch.append(
        Node(
            package="moveit_studio_agent",
            executable="move_end_effector_resampler_node",
            name="move_end_effector_resampler_node",
            parameters=[
                {
                    "input_reset_rate": 4.0,
                    "input_stop_rate": 2.0,
                    "use_sim_time": use_sim_time,
                },
            ],
            arguments=["--ros-args", "--log-level", log_level_config],
        )
    )

    # objective_library_directories_config contains a comma-separated list of directory.
    # We convert it into an array and pass it to the node.
    objective_library_directories = objective_library_directories_config.perform(
        context
    ).split(",")
    objective_server_parameters = [
        moveit_setting["robot_description_kinematics"],
        moveit_setting["robot_description_planning"],
        moveit_setting["planning_pipeline"],
        moveit_setting["sensors_3d_parameters"],
        ros_configs,
        {
            "behavior_loader_plugins": system_config_parser.get_behavior_loader_plugins(),
            "objective_library_directories": objective_library_directories,
            "behavior_status_publisher_period": 0.5,  # seconds
        },
        robot_description,
    ]

    servo_parameters = [
        moveit_setting["servo_robot_description_planning"],
        moveit_setting["servo_robot_description_kinematics"],
        moveit_setting["servo_params"],
        ros_configs,
        robot_description,
    ]

    # TODO(7664): Run as a component node instead of as a standalone node
    nodes_to_launch.append(
        Node(
            package="moveit_studio_agent",
            executable="objective_server_node_main",
            name="objective_server_node",
            output="both",
            parameters=objective_server_parameters,
            arguments=["--ros-args", "--log-level", log_level_config],
            on_exit=Shutdown(),
        )
    )

    composable_node_descriptions = []

    # Launch the robot state publisher if specified.
    # Some users may prefer to launch this alongside their drivers, so studio should provide the option
    # to override.
    launch_robot_state_publisher = system_config["hardware"].get(
        "launch_robot_state_publisher", True
    )
    if launch_robot_state_publisher:
        composable_node_descriptions.append(
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[
                    robot_description,
                    ros_configs,
                ],
            ),
        )

    # All nodes get the srdf from this publisher
    composable_node_descriptions.append(
        ComposableNode(
            package="moveit_ros_planning",
            plugin="moveit_ros_planning::SrdfPublisher",
            name="srdf_publisher",
            parameters=[robot_description_semantic, robot_description],
        )
    )

    # Include MoveIt Servo node
    composable_node_descriptions.append(
        ComposableNode(
            package="moveit_servo",
            plugin="moveit_servo::ServoNode",
            name="servo_server",
            parameters=servo_parameters,
        )
    )

    # MTC Solution Manager node
    composable_node_descriptions.append(
        ComposableNode(
            package="moveit_studio_agent",
            plugin="moveit_studio_agent::mtc_task_manager::MtcTaskManagerNode",
            name="mtc_task_manager_node",
            parameters=[
                {"use_sim_time": use_sim_time},
                robot_description,
            ],
        )
    )

    composable_node_descriptions.append(
        ComposableNode(
            package="moveit_studio_agent",
            plugin="moveit_studio_agent::planning_scene_listener::PlanningSceneListenerNode",
            name="planning_scene_listener_node",
            parameters=[
                ros_configs,
                robot_description,
            ],
        )
    )

    # Launch as much as possible in components
    nodes_to_launch.append(
        ComposableNodeContainer(
            name="moveit_studio_container",
            namespace="/",
            package="rclcpp_components",
            executable="component_container_mt",  # Use multithreaded executor
            composable_node_descriptions=composable_node_descriptions,
            output="both",
            arguments=["--ros-args", "--log-level", log_level_config],
        )
    )

    included_launch_files = []

    if is_simulated:
        included_launch_files.extend(
            [
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        system_config["simulated_hardware_launch_file_path"]
                    ),
                    launch_arguments={"is_test": is_test_config}.items(),
                )
            ]
        )

    return nodes_to_launch + included_launch_files


# A Python launch file is meant to help implement the markup based frontends like YAML and XML, and so it is declarative
# in nature rather than imperative. For this reason, it is not possible to directly access the content of
# LaunchConfiguration parameters, which are asyncio futures. To access the content of a LaunchConfiguration, we need to
# provide a context by wrapping the initialization method into an OpaqueFunction. See more info in the following links:
# https://answers.ros.org/question/397123/how-to-access-the-runtime-value-of-a-launchconfiguration-instance-within-custom-launch-code-injected-via-an-opaquefunction-in-ros2/
# https://github.com/Serafadam/interbotix_ros_manipulators/blob/xsarm_control_galactic/interbotix_ros_xsarms/interbotix_xsarm_control/launch/xsarm_control.launch.py#L31
def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="log_level",
                default_value="info",
                description="Logging level for all nodes.",
            ),
            DeclareLaunchArgument(
                "objective_library_directories",
                default_value=os.path.join(
                    os.path.abspath(get_config_folder()), "objectives"
                ),
                description="Comma separated list of directories containing objective files.",
            ),
            DeclareLaunchArgument(
                name="is_test",
                default_value="false",
                description="If true, declares that the agent is used for testing.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
