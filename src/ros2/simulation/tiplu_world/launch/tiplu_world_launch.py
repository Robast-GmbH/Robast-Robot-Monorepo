import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def create_world_urdf(context, *args, **settings):

    world_model = LaunchConfiguration('world_model').perform(context)

    # In order to swap out 'package://' paths with absolute path we need to:
    # (1) Replace the paths in the urdf text
    # (2) Write this new text into a modified_world_file
    # (3) Pass the path of the modified_world_file to Gazebo launch

    modified_world_file = os.path.join(
        get_package_share_directory("tiplu_world"), "worlds", "auto_created_gazebo_world.sdf"
    )
    with open(world_model, "r") as file:
        world_sdf = path_pattern_change_for_gazebo(file.read())
    with open(modified_world_file, "w") as file:
        file.write(world_sdf)

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            settings['gz_sim_launch'],
        ),
        launch_arguments={"gz_args": ["-r ", settings['headless'], " ", modified_world_file]}.items(),
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
            get_package_share_directory("rmf_gazebo"), "maps", "tiplu_ign" , "tiplu.world"
        ),
        description="path to the world model",
    )
    
    # Get the current Ignition file path
    ign_file_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    # Add your directory to the path
    new_directory = os.path.join(get_package_share_directory("rmf_gazebo"), "maps", "tiplu_ign" , "models")
    if new_directory not in ign_file_path.split(':'):
        ign_file_path += ':' + new_directory

    # Set the new Ignition file path
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = ign_file_path
    #export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/workspace/install/rmf_gazebo/share/rmf_gazebo/maps/tiplu_ign/models
    #/workspace_ros_gz/install/ros_gz_sim_demos/share:/workspace/src/simulation/rmf/rmf_demos:/workspace/install/rmf_gazebo/share:/workspace/install/rmf_gazebo/share/rmf_gazebo/maps/tiplu_ign/models
    
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

    # As far as I understand, to get the value of a launch argument we need a OpaqueFunction for this as described here:
    # https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file
    launch_gazebo_opaque_func = OpaqueFunction(
        function=create_world_urdf,
        kwargs={'gz_sim_launch': gz_sim_launch, 'headless': headless}
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
            "25",
            "-y",
            "-11",
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

    # opaque functions
    ld.add_action(launch_gazebo_opaque_func)

    # nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(gz_ros_bridge_cmd)

    # spawning ros2_control controller
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
