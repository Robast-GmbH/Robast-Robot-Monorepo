import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    with open("environment_vars.yaml", 'r') as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    init_x = environment_yaml["init_x"]
    init_y = environment_yaml["init_y"]
    init_yaw = environment_yaml["init_yaw"]
    config_directory = environment_yaml["config_directory"]
    is_simulation = environment_yaml["is_simulation"]

    if (is_simulation):
        map_file_yaml = os.path.join(get_package_share_directory(
            'heiligenhafen_commander'), 'heiligenhafen_map.yaml')
        use_sim_time_default_value = 'true'
    else:
        map_file_yaml = os.path.join(get_package_share_directory(
            'heiligenhafen_commander'), 'heiligenhafen_map.yaml')
        use_sim_time_default_value = 'false'

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')

    amcl_launch_file = os.path.join(get_package_share_directory(
        'robast_nav_launch'), 'launch', 'amcl_launch.py')

    amcl_arguments = {
        'map_file': map_file,
        'use_sim_time': use_sim_time,
        'namespace': namespace
    }.items()

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time_default_value,
        description='Use simulation (Gazebo) clock if true')

    map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=map_file_yaml,
        description='map server map file to load')

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch_file), launch_arguments=amcl_arguments)

    # start the demo autonomy task
    demo_cmd = Node(
        package='heiligenhafen_commander',
        executable='main',
        emulate_tty=True,
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(map_file_cmd)
    ld.add_action(amcl_launch)
    ld.add_action(demo_cmd)
    return ld
