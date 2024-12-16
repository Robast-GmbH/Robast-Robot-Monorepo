import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('drawer_sm')
    bt_config_params = LaunchConfiguration('bt_config_params')

    declare_bt_config_params_cmd = DeclareLaunchArgument(
        'bt_config_params',
        default_value=os.path.join(bringup_dir, 'config', 'heartbeat_tree_params.yaml'),
        description='path to the bt_params.yaml file')

    params = {
        "bt_path": os.path.join(bringup_dir, "trees", "heartbeat.xml"),
        "plugins": ["heartbeat_condition_node", "robast_error_pub_node", "publish_string_topic_action_node"],
        "trigger_topic": "trigger_heartbeat_tree",
        "main_tree": "default_heartbeat_drawer_tree",
        "tree_tick_time": 10
    }

    bt_node = Node(
        package="bt_base_nodes",
        executable="heartbeat_tree_initiator",
        name="heartbeat_tree_initiator",
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        parameters=[params])
    
    ld = LaunchDescription()
    ld.add_action(declare_bt_config_params_cmd)
    ld.add_action(bt_node)
    return ld
