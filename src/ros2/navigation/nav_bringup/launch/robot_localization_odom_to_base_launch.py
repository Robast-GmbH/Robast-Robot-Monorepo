import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
            
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
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

    
    if os.environ["is_simulation"] == 'True':
        #TODO @all check how to rewrite a matrix parameter. In this case odom0
        use_sim_time_default = "true"
        param_substitutions = {
            'use_sim_time': use_sim_time_default,
            "odom_frame": "odom",
            "base_link_frame":"base_footprint",
            "world_frame":"odom",
            "odom0": "diff_drive_base_controller/odom",
            "imu0": "imu/data",
            "odom0_differential": "true",
            "odom0_relative": "false"
        }
        sim_params = os.path.join(
            get_package_share_directory("nav_bringup"),
            "config_simulation",
            "localization",
            "robot_localization_ekf_odom_to_base.yaml",
        ),
        configured_params = RewrittenYaml(
            source_file=sim_params,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)
    else:
        param_substitutions = {
            'use_sim_time': use_sim_time,
            "odom_frame": "robot/odom",
            "base_link_frame":"robot/base_footprint",
            "world_frame":"robot/odom",
            "odom0": "/robot/robotnik_base_control/odom",
            "imu0": "robot/imu/data",
        }
        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[configured_params],
    )

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(robot_localization_node)

    return ld
