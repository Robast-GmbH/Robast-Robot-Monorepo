import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    with open("environment_vars.yaml", 'r') as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
            print(environment_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_ros_bridge_yaml = os.path.join(get_package_share_directory('tiplu_world'), 'config', 'gz_ros_bridge.yaml')

    robot_xml = xacro.process_file(os.path.join(get_package_share_directory(
        'rb_theron_description'), 'robots', environment_yaml["robot"]+'.urdf.xacro'), mappings={'prefix': environment_yaml["prefix"]}).toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_model = LaunchConfiguration('world_model')
    robot_name = LaunchConfiguration('robot_name')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='rb_theron',
        description='name of the robot in the simulation'
    )

    declare_world_model_cmd = DeclareLaunchArgument(
        'world_model',
        default_value=os.path.join(get_package_share_directory('tiplu_world'), 'worlds', '6OG' + '.sdf'),
        description='path to the world model'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='whether to use sim time or not'
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_xml}
        ],
        output="screen",
    )

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': world_model}.items(),
    )

    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-z', "0.1",
        ],
        output='screen',
    )

    gz_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
                {'config_file': gz_ros_bridge_yaml},
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_model_cmd)
    ld.add_action(declare_robot_model_cmd)

    # included launches
    ld.add_action(gz_sim_cmd)

    # nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(gz_ros_bridge_cmd)

    return ld