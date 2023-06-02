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
    use_sim_time = LaunchConfiguration('use_sim_time')
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
    
        #TODo fix the Realtime multi robot issue
        # if environment_yaml["is_simulation"]:
    use_sim_time_default = "true"
    robot_namespace="rb0"
    param_substitutions = {
            'use_sim_time': use_sim_time_default,
            "odom_frame": "rb0/odom",
            "base_link_frame": "rb0/base_footprint",
            "world_frame": "rb0/odom",
            "odom0": "/rb0/odom",
            "imu0": "/rb0/imu/data"        
            }
    # param_substitutions2 = {
    #         'use_sim_time': use_sim_time_default,
    #         "odom_frame": "rb1/odom",
    #         "base_link_frame": "rb0/base_footprint",
    #         "world_frame": "rb1/odom",
    #         "odom0": "/rb0/odom",
    #         "imu0": "/rb0/imu/data"        
    #         }
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
    
    # configured_params2 = RewrittenYaml(
    #         source_file=params_file,
    #         root_key=namespace2,
    #         param_rewrites=param_substitutions2,
    #         convert_types=True)

    robot_localization_node = Node(
            namespace=namespace,
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[configured_params],
        )
    
    # robot_localization_node2 = Node(
    #         namespace=namespace2,
    #         package="robot_localization",
    #         executable="ekf_node",
    #         name="ekf_filter_node",
    #         output="screen",
    #         parameters=[configured_params2],
    #     )


    ld = LaunchDescription()

        # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

        # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
        # Add the actions to launch all of the navigation nodes
    ld.add_action(robot_localization_node)
    # ld.add_action(robot_localization_node2)

    return ld