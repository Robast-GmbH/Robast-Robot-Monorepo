import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)


def launch_setup(context, *args, **settings):
    bringup_dir = get_package_share_directory("drawer_sm")
    id = LaunchConfiguration("id").perform(context)

    node_name = "heartbeat_tree_initiator_" + id

    params = {
        "bt_path": os.path.join(bringup_dir, "trees", "heartbeat.xml"),
        "plugins": ["heartbeat_condition_node", "robast_error_pub_node"],
        "trigger_topic": "trigger_heartbeat_tree",
        "main_tree": "default_heartbeat_drawer_tree",
        "tree_tick_time": 10,
        "id": id,
    }

    bt_node = Node(
        package="bt_base_nodes",
        executable="heartbeat_tree_initiator",
        name=node_name,
        output="screen",
        parameters=[params],
    )

    return [bt_node]


def generate_launch_description():
    bringup_dir = get_package_share_directory("drawer_sm")

    declare_bt_config_params_cmd = DeclareLaunchArgument(
        "bt_config_params",
        default_value=os.path.join(bringup_dir, "config", "heartbeat_params.yaml"),
        description="path to the bt_params.yaml file",
    )

    declare_id_cmd = DeclareLaunchArgument(
        "id",
        default_value="1",
        description="Id of device to track the heartbeat of",
    )

    launch_setup_opaque_func = OpaqueFunction(function=launch_setup)

    ld = LaunchDescription()
    ld.add_action(declare_bt_config_params_cmd)
    ld.add_action(declare_id_cmd)

    ld.add_action(launch_setup_opaque_func)

    return ld
