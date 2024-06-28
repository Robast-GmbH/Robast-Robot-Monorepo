import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    OpaqueFunction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition


# TODO @all: There are still some small issues with loading packages,
# TODO @all: but it starts and seems to work fine.


def launch_robot_state_publisher(context, *args, **settings):
    position_joint_type = LaunchConfiguration("position_joint_type").perform(context)
    model_door_opening_mechanism = LaunchConfiguration(
        "model_door_opening_mechanism"
    ).perform(context)

    robot_xml = xacro.process_file(
        os.path.join(
            get_package_share_directory("rb_theron_description"),
            "robots",
            settings["robot"] + ".urdf.xacro",
        ),
        mappings={
            "prefix": settings["prefix"],
            "ros2_control_hardware_type": "gz_ros2_control",
            "ros2_control_hardware_type_positon_joint": "gz_ros2_control",
            "ros_distro": settings["ros_distro"],
            "position_joint_type": position_joint_type,
            "model_door_opening_mechanism": model_door_opening_mechanism,
        },
    ).toxml()

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"use_sim_time": settings["use_sim_time"]},
            {"robot_description": robot_xml},
        ],
        output="screen",
    )

    return [start_robot_state_publisher_cmd]


def create_world_urdf(context, *args, **settings):
    world_model = LaunchConfiguration("world_model").perform(context)

    # In order to swap out 'package://' paths with absolute path we need to:
    # (1) Replace the paths in the urdf text
    # (2) Write this new text into a modified_world_file
    # (3) Pass the path of the modified_world_file to Gazebo launch

    modified_world_file = os.path.join(
        get_package_share_directory("tiplu_world"),
        "worlds",
        "auto_created_gazebo_world.sdf",
    )
    with open(world_model, "r") as file:
        world_sdf = path_pattern_change_for_gazebo(file.read())
    with open(modified_world_file, "w") as file:
        file.write(world_sdf)

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            settings["gz_sim_launch"],
        ),
        launch_arguments={
            "gz_args": ["-r ", settings["headless"], " ", modified_world_file]
        }.items(),
    )

    return [gz_sim_cmd]


def path_pattern_change_for_gazebo(urdf_string):
    """
    Replaces strings in a URDF file such as
        package://package_name/path/to/file
    to the actual full path of the file.
    """
    data = urdf_string
    package_expressions = re.findall("(package://([^//]*))", data)
    for expr in set(package_expressions):
        data = data.replace(expr[0], ("file://" + get_package_share_directory(expr[1])))

    return data


def generate_launch_description():
    ros_distro = os.environ["ROS_DISTRO"]
    gz_version = os.environ["GZ_VERSION"]

    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    robot_name = LaunchConfiguration("robot_name")
    position_joint_type = LaunchConfiguration("position_joint_type")
    model_door_opening_mechanism = LaunchConfiguration("model_door_opening_mechanism")
    init_x = os.environ["init_x"]
    init_y = os.environ["init_y"]
    init_yaw = os.environ["init_yaw"]
    prefix = os.environ["prefix"]
    robot = os.environ["robot"]

    if gz_version == "fortress":
        pkg_ros_gz_sim = get_package_share_directory("ros_ign_gazebo")
        gz_sim_launch = os.path.join(pkg_ros_gz_sim, "launch", "ign_gazebo.launch.py")
        gz_ros_bridge_yaml = os.path.join(
            get_package_share_directory("tiplu_world"), "config", "ign_ros_bridge.yaml"
        )
    if gz_version == "garden" or gz_version == "harmonic":
        pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
        gz_sim_launch = os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        gz_ros_bridge_yaml = os.path.join(
            get_package_share_directory("tiplu_world"), "config", "gz_ros_bridge.yaml"
        )

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
        description=(
            "path to the world model. Alternative: "
            "get_package_share_directory(rmf_gazebo), "
            "maps, tiplu_ign , tiplu.world"
        ),
    )

    # Add world/models to the path
    ign_resource_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    world_path = os.path.join(
        get_package_share_directory("rmf_gazebo"), "maps", "tiplu_ign", "tiplu.world"
    )
    if world_path not in ign_resource_path.split(":"):
        ign_resource_path += ":" + world_path
    world_path = os.path.join(
        get_package_share_directory("rmf_gazebo"), "maps", "tiplu_ign", "models"
    )
    if world_path not in ign_resource_path.split(":"):
        ign_resource_path += ":" + world_path
    world_path = os.path.join(
        get_package_share_directory("rmf_gazebo"), "maps", "tiplu", "models"
    )
    if world_path not in ign_resource_path.split(":"):
        ign_resource_path += ":" + world_path
    world_path = os.path.join(
        get_package_share_directory("tiplu_world"), "models", "6_OG_normal_doors"
    )
    if world_path not in ign_resource_path.split(":"):
        ign_resource_path += ":" + world_path
    world_path = os.path.join(os.environ["HOME"], ".gazebo", "models")
    if world_path not in ign_resource_path.split(":"):
        ign_resource_path += ":" + world_path
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ign_resource_path

    # Add plugins to the path
    ign_plugin_path = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")
    world_path = os.path.join(
        get_package_share_directory("rmf_robot_sim_gz_plugins"),
        "lib",
        "rmf_robot_sim_gz_plugins",
    )
    if world_path not in ign_plugin_path.split(":"):
        ign_plugin_path += ":" + world_path
    world_path = os.path.join(
        get_package_share_directory("rmf_building_sim_gz_plugins"),
        "lib",
        "rmf_building_sim_gz_plugins",
    )
    if world_path not in ign_plugin_path.split(":"):
        ign_plugin_path += ":" + world_path
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = ign_plugin_path
    os.environ["IGN_GUI_PLUGIN_PATH"] = ign_plugin_path

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

    declare_position_joint_type_cmd = DeclareLaunchArgument(
        "position_joint_type",
        default_value="fixed",
        description="whether to model the position joint as fixed, prismatic or planar",
    )

    declare_model_door_opening_mechanism_cmd = DeclareLaunchArgument(
        "model_door_opening_mechanism",
        default_value="False",
        description="whether to model the door opening mechanism",
    )

    # As far as I understand, to get the value of a launch argument
    # we need a OpaqueFunction for this as described here:
    # https://t.ly/pGVnN
    launch_gazebo_opaque_func = OpaqueFunction(
        function=create_world_urdf,
        kwargs={"gz_sim_launch": gz_sim_launch, "headless": headless},
    )

    launch_robot_state_publisher_opaque_func = OpaqueFunction(
        function=launch_robot_state_publisher,
        kwargs={
            "use_sim_time": use_sim_time,
            "robot": robot,
            "prefix": prefix,
            "ros_distro": ros_distro,
        },
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
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
            "--use-sim-time",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
            "--use-sim-time",
        ],
        output="screen",
    )

    load_mobile_base_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "mobile_base_controller_cmd_vel",
            "--use-sim-time",
        ],
        output="screen",
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_base_controller",
            "--use-sim-time",
        ],
        output="screen",
    )

    load_drawer_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "drawer_joint_trajectory_controller",
            "--use-sim-time",
        ],
        output="screen",
    )

    spawn_ros2_controller_without_arm = GroupAction(
        condition=UnlessCondition(model_door_opening_mechanism),
        actions=[
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot_cmd,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_diff_drive_base_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_diff_drive_base_controller,
                    on_exit=[load_drawer_joint_trajectory_controller],
                )
            ),
        ],
    )

    spawn_ros2_controller_with_arm = GroupAction(
        condition=IfCondition(PythonExpression([model_door_opening_mechanism])),
        actions=[
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot_cmd,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_mobile_base_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_mobile_base_controller,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_trajectory_controller,
                    on_exit=[load_diff_drive_base_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_diff_drive_base_controller,
                    on_exit=[load_drawer_joint_trajectory_controller],
                )
            ),
        ],
    )

    ld = LaunchDescription()

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_position_joint_type_cmd)
    ld.add_action(declare_model_door_opening_mechanism_cmd)
    ld.add_action(declare_world_model_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_headless_cmd)

    # opaque functions
    ld.add_action(launch_gazebo_opaque_func)
    ld.add_action(launch_robot_state_publisher_opaque_func)

    # nodes
    ld.add_action(spawn_robot_cmd)
    ld.add_action(gz_ros_bridge_cmd)

    # spawn ros2_controllers
    ld.add_action(spawn_ros2_controller_without_arm)
    ld.add_action(spawn_ros2_controller_with_arm)

    return ld
