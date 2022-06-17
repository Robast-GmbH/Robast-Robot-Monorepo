import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    if(os.environ['ROS_DISTRO'] == 'galactic'):
        nav2_params_yaml = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'config', 'nav2_params_galactic.yaml')
    else:
        nav2_params_yaml = os.path.join(get_package_share_directory('robast_nav_launch'), 'config', 'nav2_params.yaml')

    nav2_localization_params_yaml = os.path.join(get_package_share_directory(
        'robast_nav_launch'), 'config', 'localization_params.yaml')
    # map_file = os.path.join(get_package_share_directory('robast_nav_launch'), 'maps', '5OG.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_file = LaunchConfiguration('map_file')
    map_server_map_topic = LaunchConfiguration('map_server_map_topic')
    amcl_map_topic = LaunchConfiguration('amcl_map_topic')

    # Mind the order in which the nodes are started!!!
    lifecycle_nodes = [
        'map_server',
        'amcl',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory('robast_nav_launch'), 'maps','Tiplu_6OG_2', 'Tiplu_6OG_2.yaml'),
        description='map server map file to load')

    # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    remappings_amcl = [('/robot/tf', 'tf'),
                       ('/robot/tf_static', 'tf_static'),
                       #('/robot/map', '/map')
                       ]

    remappings_map_server = remappings_amcl

    # Make re-written yaml
    param_substitutions = {
        'namespace': namespace,
        'base_frame_id': 'robot_base_link',
        'global_frame_id': 'robot_map',
        'odom_frame_id': 'robot_odom',
        'scan_topic': 'front_laser/scan',
        'map_topic': 'map'}

    configured_params = RewrittenYaml(
        source_file=nav2_amcl_params_yaml,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file},
                    {'frame_id': 'robot_map'}
                    ],
        remappings=remappings_map_server)

    start_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        namespace=namespace,
        parameters=[
            configured_params,
            {'use_sim_time': use_sim_time},
            {"initial_pose": {"x": float(0), "y": float(0), "yaw": 0.0}},
            {"set_initial_pose": True},
        ],
        remappings=remappings_amcl)

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    ld = LaunchDescription()
    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # arguments
    ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(map_file_cmd)

    # nodes
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_map_server_cmd)

    return ld
