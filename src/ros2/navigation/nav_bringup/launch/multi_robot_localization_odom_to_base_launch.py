import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    with open("/workspace/src/navigation/environment_vars.yaml", "r") as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='rb0',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value="false",
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("nav_bringup"),
            "config",
            "localization",
            "robot_localization_ekf_odom_to_base.yaml",
        ),
        description="Path to the localization config file",
    )

    # TODo Torben fix the Realtime multi robot issue
    # if environment_yaml["is_simulation"]:
    robot_locatisation_Nodes = []
    use_sim_time_default = "true"
    for robot_namespace in environment_yaml["robot_namspaces"]:
        param_substitutions = {
            'use_sim_time': use_sim_time_default,
            "odom_frame": robot_namespace+"/odom",
            "base_link_frame": robot_namespace+"/base_footprint",
            "world_frame": robot_namespace+"/odom",
            "odom0": "/"+robot_namespace+"/odom",
            "imu0": "/"+robot_namespace+"/imu/data"
            }

        # else:
        #     param_substitutions = {
        #         'use_sim_time': use_sim_time,
        #         "odom_frame": "robot_odom",
        #         "base_link_frame":"robot_base_footprint",
        #         "world_frame":"robot_odom",
        #         "odom0": "/robot/robotnik_base_control/odom",
        #         "imu0": "/robot/imu/data"
        #     }

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

        robot_localization_node = Node(
            namespace=namespace,
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[configured_params],
        )
        robot_locatisation_Nodes.append(robot_localization_node)

    ld = LaunchDescription()

    #  Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    #  Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    #  Add the actions to launch all of the navigation nodes
    for robot_locatisation_Node in robot_locatisation_Nodes:
        ld.add_action(robot_locatisation_Node)

    return ld
