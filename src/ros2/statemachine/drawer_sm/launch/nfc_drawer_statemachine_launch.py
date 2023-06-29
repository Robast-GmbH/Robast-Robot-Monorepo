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
        default_value=os.path.join(bringup_dir, 'config', 'nfc_bt_params.yaml'),
        description='path to the bt_params.yaml file')

    bt_node = Node(
        package="bt_base_nodes",
        executable="drawer_nfc_tree_initiator",
        name="drawer_nfc_tree_initiator",
        output="screen",
        parameters=[bt_config_params])
    ld = LaunchDescription()
    ld.add_action(declare_bt_config_params_cmd)
    ld.add_action(bt_node)
    return ld