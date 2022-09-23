import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# ros2 run nav2_map_server map_saver_cli -t slam_map -f test

WORLD_MODEL = os.environ['WORLD_MODEL']
POSE_INIT_X = os.environ['POSE_INIT_X']
POSE_INIT_Y = os.environ['POSE_INIT_Y']
POSE_INIT_Z = os.environ['POSE_INIT_Z']


def generate_launch_description():

    nav_bringup_dir = get_package_share_directory('nav_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_posegraph = LaunchConfiguration('slam_posegraph')
    # robot_start_pose = LaunchConfiguration('map_start_pose')
    slam_mode = LaunchConfiguration('slam_mode')
    slam_map_topic = LaunchConfiguration('slam_map_topic')
    transform_publish_period = LaunchConfiguration('transform_publish_period')
    slam_executable = LaunchConfiguration('slam_executable')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    start_parallel_mapping = Node(
        package='robast_map_update_module',
        executable='map_combine_node',
        name='map_combine_node',
        output='screen',
        # parameters=[
        #     {'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(start_parallel_mapping)
    return ld
