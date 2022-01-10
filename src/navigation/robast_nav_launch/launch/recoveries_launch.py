import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    if(os.environ['ROS_DISTRO'] == 'galactic'):
        recoveries_params_yaml = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'config', 'recoveries_params.yaml')
    else:
        recoveries_params_yaml = os.path.join(get_package_share_directory(
            'robast_nav_launch'), 'config', 'recoveries_params.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    costmap_namespace = LaunchConfiguration('costmap_namespace')
    recoveries_params_file = LaunchConfiguration('recoveries_params_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    costmap_namespace_cmd = DeclareLaunchArgument(
        'costmap_namespace',
        default_value='recoveries_costmap',
        description='Namespace that the costmap topic is published in. Attention: The costmap_namespace defines also the naming for the node that needs to be stated before the ros__parameters in the yaml parameter file.')

    declare_recoveries_params_file_cmd = DeclareLaunchArgument(
        'recoveries_params_file',
        default_value=recoveries_params_yaml,
        description='Full path to the ROS 2 recoveries params file to use')

    lifecycle_nodes = [
        'recoveries_costmap',
        'recoveries_server',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    recoveries_costmap_cmd = Node(
        package='robast_nav_recoveries',
        executable='recoveries_costmap',
        name='recoveries_costmap',
        output='screen',
        parameters=[recoveries_params_yaml,
                    {"costmap_namespace": costmap_namespace}],
        remappings=remappings)

    recoveries_server_cmd = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[recoveries_params_yaml],
        remappings=remappings)

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_recoveries',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    ld = LaunchDescription()
    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(costmap_namespace_cmd)
    ld.add_action(declare_recoveries_params_file_cmd)

    # nodes
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(recoveries_costmap_cmd)
    ld.add_action(recoveries_server_cmd)

    return ld
