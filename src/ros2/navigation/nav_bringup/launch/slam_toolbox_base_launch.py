import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# ros2 run nav2_map_server map_saver_cli -t slam_map -f test


def generate_launch_description():
    init_x = None
    init_y = None
    init_yaw = None

    if os.path.isfile('/logs/last_pose.yaml'):
        with open('/logs/last_pose.yaml', 'r') as file:
            config = yaml.safe_load(file)
            init_x = config['map_pose']['position']['x']
            init_y = config['map_pose']['position']['y']
            init_yaw = config['map_pose']['orientation']['yaw']
    else:
        init_x = os.getenv('init_x')
        init_y = os.getenv('init_y')
        init_yaw = os.getenv('init_yaw')
    
    nav_bringup_dir = get_package_share_directory("nav_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    slam_params_file = LaunchConfiguration("slam_params_file")
    slam_posegraph = LaunchConfiguration("slam_posegraph")
    # robot_start_pose = LaunchConfiguration('map_start_pose')
    slam_mode = LaunchConfiguration("slam_mode")
    slam_map_topic = LaunchConfiguration("slam_map_topic")
    transform_publish_period = LaunchConfiguration("transform_publish_period")
    slam_executable = LaunchConfiguration("slam_executable")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_slam_toolbox_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            nav_bringup_dir, "config", "slam", "slam_toolbox_params_offline.yaml"
        ),
        description="Full path to the ROS 2 slam params file to use",
    )

    declare_slam_posegraph_file_cmd = DeclareLaunchArgument(
        "slam_posegraph",
        default_value=os.path.join(nav_bringup_dir, "maps", "new_6OG"),
        description="Full path to the slam_toolbox posegraph map file to use",
    )


    declare_slam_mode_param_cmd = DeclareLaunchArgument(
        "slam_mode",
        default_value="mapping",
        description="You can choose between mapping and localization",
    )

    declare_slam_map_topic_cmd = DeclareLaunchArgument(
        "slam_map_topic",
        default_value="map",
        description="Name of the occupancy grid topic e.g. /map",
    )

    declare_transform_publish_period_cmd = DeclareLaunchArgument(
        "transform_publish_period",
        default_value="0.02",
        description="if 0 never publishes odometry Wenn der zuhoch => nav ist nurnoch in recovery",
    )

    declare_slam_executable_cmd = DeclareLaunchArgument(
        "slam_executable",
        default_value="sync_slam_toolbox_node",
        description="kind of slam toolbox node",
    )

    # Make re-written yaml
    param_substitutions = {
        "namespace": namespace,
        "map_file_name": slam_posegraph,
        # 'use_sim_time': use_sim_time,
        "mode": slam_mode,
        "map_name": slam_map_topic,
        "transform_publish_period": transform_publish_period,
    }

    configured_params = RewrittenYaml(
        source_file=slam_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    remappings_amcl = [("/map", "map"), ("/map_metadata", "map_metadata")]

    remappings_map_server = remappings_amcl

    start_slam_toolbox_cmd = Node(
        package="slam_toolbox",
        executable=slam_executable,
        namespace=namespace,
        name="slam_toolbox",
        output="screen",
        parameters=[
            configured_params,
            {"map_file_name": slam_posegraph},
            {"use_sim_time": use_sim_time},
            {"map_start_pose": [float(init_x), float(init_y), float(init_yaw)]},
        ],
        remappings=remappings_map_server,
    )

    ld = LaunchDescription()
    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slam_toolbox_params_file_cmd)
    ld.add_action(declare_slam_posegraph_file_cmd)
    ld.add_action(declare_slam_mode_param_cmd)
    ld.add_action(declare_slam_map_topic_cmd)
    ld.add_action(declare_transform_publish_period_cmd)
    ld.add_action(declare_slam_executable_cmd)
    # ld.add_action(declare_robot_start_pose_cmd)

    # nodes
    ld.add_action(start_slam_toolbox_cmd)
    return ld
