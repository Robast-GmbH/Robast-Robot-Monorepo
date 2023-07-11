import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bt_node = ExecuteProcess(
        #     cmd=['ros2', 'bag', 'record', '-o', '/workspace/bags/robot_imu.db3', 'imu/data', 'tf', 'tf_static', 'odom', 'clock'],
                cmd=['ros2', 'bag', 'record', '-o', '/workspace/bags/robot_imu.db3', 
                     'tf', 
                     'tf_static', 
                     'robot/robotnik_base_control/odom', 
                     'robot/imu/data', 
                     'robot/vectornav/imu/data',
                     'robot/pad_teleop/cmd_vel',
                     'robot/vectornav/odom',
                     'clock'],
                output='screen'
        )
    ld = LaunchDescription()
    ld.add_action(bt_node)
    return ld