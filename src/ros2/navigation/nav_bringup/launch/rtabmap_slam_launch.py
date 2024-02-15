import os

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
     
    config_directory = os.environ['config_directory']
    is_simulation = os.environ['is_simulation']

    namespace = LaunchConfiguration("namespace")
    nav_bringup_dir = get_package_share_directory("nav_bringup")

    rtabmap_slam_params_yaml = os.path.join(
        nav_bringup_dir,
        config_directory,
        "slam",
        "rtabmap_slam_params_mapping.yaml"
    )

    Node(
            package='rtabmap_odom', executable='stereo_odometry', name="stereo_odometry", output="screen",
            parameters = rtabmap_slam_params_yaml,
            remappings=[
                ("left/image_rect", LaunchConfiguration('left_image_topic_relay')),
                ("right/image_rect", LaunchConfiguration('right_image_topic_relay')),
                ("left/camera_info", LaunchConfiguration('left_camera_info_topic')),
                ("right/camera_info", LaunchConfiguration('right_camera_info_topic')),
                ("rgbd_image", LaunchConfiguration('rgbd_topic_relay')),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic'))],
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args")],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),