import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('drawer_sm')
    bt_config_params = LaunchConfiguration('bt_config_params')

    declare_bt_config_params_cmd = DeclareLaunchArgument(
        'bt_config_params',
        default_value=os.path.join(bringup_dir, 'config', 'heartbeat_tree_params.yaml'),
        description='path to the bt_params.yaml file')

    param_substitutions = {
        'bt_path': os.path.join(bringup_dir, 'trees', 'heartbeat.xml')}
    
    configured_params = RewrittenYaml(
        source_file=bt_config_params,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    bt_node = Node(
        package="bt_base_nodes",
        executable="heartbeat_tree_initiator",
        name="heartbeat_tree_initiator",
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        parameters=[configured_params])
    
    ld = LaunchDescription()
    ld.add_action(declare_bt_config_params_cmd)
    ld.add_action(bt_node)
    return ld
