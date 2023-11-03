import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    ros_distro = os.environ["ROS_DISTRO"]
    gz_version = os.environ["GZ_VERSION"]

    if (gz_version == "fortress"):
        pkg_ros_gz_sim = get_package_share_directory("ros_ign_gazebo")
        gz_sim_launch = os.path.join(pkg_ros_gz_sim, "launch", "ign_gazebo.launch.py")
        gz_ros_bridge_yaml = os.path.join(get_package_share_directory("tiplu_world"), "config", "ign_ros_bridge.yaml")
    if (gz_version == "garden" or gz_version == "harmonic"):
        pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
        gz_sim_launch = os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        gz_ros_bridge_yaml = os.path.join(get_package_share_directory("tiplu_world"), "config", "gz_ros_bridge.yaml")

    robot_xml = xacro.process_file(
        os.path.join(
            get_package_share_directory("rb_theron_description"),
            "robots",
            os.environ["robot"] + ".urdf.xacro",
        ),
        mappings={"prefix": os.environ["prefix"],
                  "ros2_control_hardware_type": "gz_ros2_control",
                  "ros_distro": ros_distro},
    ).toxml()

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_model = LaunchConfiguration("world_model")
    headless = LaunchConfiguration("headless")
    robot_name = LaunchConfiguration("robot_name")
    init_x = os.environ['init_x']
    init_y = os.environ["init_y"]
    init_yaw = os.environ["init_yaw"]

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_robot_model_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="rb_theron",
        description="name of the robot in the simulation",
    )

    declare_world_model_cmd = DeclareLaunchArgument(
        "world_model",
        default_value=os.path.join(
            get_package_share_directory("tiplu_world"), "worlds", "6OG" + ".sdf"
        ),
        description="path to the world model",
    )
    
    declare_headless_cmd = DeclareLaunchArgument(
        "headless",
        default_value=" ",
        description="Weather to run in headless mode (-s) or with gui ''",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="whether to use sim time or not",
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}, {"robot_description": robot_xml}],
        output="screen",
    )

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gz_sim_launch,
        ),
        launch_arguments={"gz_args": ["-r ", headless, " ", world_model]}.items(),
    )

    spawn_robot_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-z",
            "0.2",
            "-x",
            init_x,
            "-y",
            init_y,
            "-Y",
            init_yaw,
        ],
        output="screen",
    )

    gz_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": gz_ros_bridge_yaml},
        ],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster',
             '--use-sim-time'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller',
             '--use-sim-time'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller',
             '--use-sim-time'],
        output='screen'
    )

    load_drawer_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'drawer_joint_trajectory_controller',
             '--use-sim-time'],
        output='screen'
    )

    ld = LaunchDescription()

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_model_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_headless_cmd)

    # included launches
    ld.add_action(gz_sim_cmd)

    # nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(gz_ros_bridge_cmd)

    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot_cmd,
                on_exit=[load_joint_state_broadcaster],
            )
        ))
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ))
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_diff_drive_base_controller],
            )
        ))
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_drive_base_controller,
                on_exit=[load_drawer_joint_trajectory_controller],
            )
        ))

    return ld
