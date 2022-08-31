import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    with open("environment_vars.yaml", 'r') as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)
    robot_xml = xacro.process_file(os.path.join(get_package_share_directory(
        'rb_theron_description'), 'robots', environment_yaml["robot"]+'.urdf.xacro'), mappings={'prefix': environment_yaml["prefix"]}).toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    init_x = LaunchConfiguration('init_x', default="0.0")
    init_y = LaunchConfiguration('init_y', default="0.0")
    init_yaw = LaunchConfiguration('init_yaw', default="0.0")
    world_model = LaunchConfiguration('world_model')
    robot_name = LaunchConfiguration('robot_name')
    extra_gazebo_args = LaunchConfiguration('extra_gazebo_args')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    run_gzclient_cmd = DeclareLaunchArgument(
        'run_gzclient',
        default_value='True',
        description='Top-level namespace'
    )

    declare_world_model_cmd = DeclareLaunchArgument(
        'world_model',
        default_value=os.path.join(get_package_share_directory('heiligenhafen'),
                                   'worlds', 'HH-Building', 'HH-Building_world' + '.model'),
        description='path to the world model'
    )

    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='rb_theron',
        description='name of the robot in the simulation')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='whether to use sim time or not')

    declare_extra_gazebo_args_cmd = DeclareLaunchArgument(
        'extra_gazebo_args',
        default_value='',
        description='extra_gazebo_args e.g. --lockstep'
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                {'use_sim_time': use_sim_time},
                # {'robot_description': Command(['xacro', ' ', xacro_path, ' gazebo:=False'])}
                {'robot_description': robot_xml}
        ],
        output="screen")

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    start_gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_model, 'extra_gazebo_args': extra_gazebo_args}.items(),
    )

    start_gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration("run_gzclient"))
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('heiligenhafen'), 'launch', 'spawn_robot_launch.py')
        ),
        launch_arguments={'init_x': init_x, 'init_y': init_y, 'init_yaw': init_yaw, 'robot_name': robot_name}.items(),
    )

    print('world_file_path : {}'.format(world_model))

    ld = LaunchDescription()

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_model_cmd)
    ld.add_action(run_gzclient_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_extra_gazebo_args_cmd)

    # included launches
    ld.add_action(start_gzserver_cmd)
    ld.add_action(start_gzclient_cmd)

    # nodes
    ld.add_action(start_robot_state_publisher_cmd)

    # late execute
    ld.add_action(spawn_robot_cmd)

    return ld

    # launch_file_dir = os.path.join(get_package_share_directory('tiplu_world'), 'launch')
    # rb_theron_controller_dir = os.path.join(get_package_share_directory(
    #     'rb_theron_description'), 'config', 'rb_theron_controller.yaml')

    # Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {"robot_description": get_robot_xml()},
    #         {'use_sim_time': use_sim_time},
    #         rb_theron_controller_dir],
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     },
    # ),

    # Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["diffbot_base_controller"],
    #     output="screen",
    # ),

    # Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_state_broadcaster"],
    #     output="screen",
    # ),
