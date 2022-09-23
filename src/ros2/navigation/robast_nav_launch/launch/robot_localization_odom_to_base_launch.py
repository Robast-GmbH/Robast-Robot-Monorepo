import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    with open("environment_vars.yaml", "r") as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    config_directory = environment_yaml["config_directory"]

    robot_localization_params_yaml = os.path.join(
        get_package_share_directory("robast_nav_launch"),
        config_directory,
        "localization",
        "robot_localization_ekf_odom_to_base.yaml",
    )

    # Please mind:
    # Because we have config files for simulation and real life we set use_sim_time directly in the config files

    # namespace = LaunchConfiguration('namespace')
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # declare_namespace_cmd = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='',
    #     description='Top-level namespace'
    # )
    # declare_use_sim_time_cmd = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='true',
    #     description='Use simulation (Gazebo) clock if true')
    # param_substitutions = {
    #     'use_sim_time': use_sim_time
    # }
    # configured_params = RewrittenYaml(
    #     source_file=robot_localization_params_yaml,
    #     root_key=namespace,
    #     param_rewrites=param_substitutions,
    #     convert_types=True)

    start_robot_localization_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[robot_localization_params_yaml],
    )

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    # ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(start_robot_localization_cmd)

    return ld
