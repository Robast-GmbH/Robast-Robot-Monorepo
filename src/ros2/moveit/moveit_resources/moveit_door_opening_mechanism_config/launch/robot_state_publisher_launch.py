import os
import atexit
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

def launch_robot_state_publisher(context, *args, **settings):
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = use_sim_time_str.lower() == "true"

    prefix = LaunchConfiguration("prefix").perform(context)
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type").perform(context)
    ros2_control_hardware_type_positon_joint = LaunchConfiguration("ros2_control_hardware_type_positon_joint").perform(context)
    robot_description_path = LaunchConfiguration("robot_description_path").perform(context)

    robot_xml = xacro.process_file(
        robot_description_path,
        mappings={
            "ros2_control_hardware_type": ros2_control_hardware_type,
            "ros2_control_hardware_type_positon_joint": ros2_control_hardware_type_positon_joint,
            "ros_distro": os.environ["ROS_DISTRO"],
            "prefix": prefix,
        },
    ).toxml()

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_xml},
        ],
        output="screen",
    )

    return start_robot_state_publisher_cmd


def generate_launch_description():

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Whether to use simulation (Gazebo) clock"
    )

    declare_prefix_cmd = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description="The prefix to be used for the robot description",
    )

    declare_ros2_control_hardware_type_cmd = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="dryve_d1",
        description="The hardware type to use for ros2 control",
    )

    declare_ros2_control_hardware_type_positon_joint_cmd = DeclareLaunchArgument(
        "ros2_control_hardware_type_positon_joint",
        default_value="real_life",
        description="The hardware type to use for ros2 control for position joint",
    )

    declare_robot_description_path_cmd = DeclareLaunchArgument(
        "robot_description_path",
        default_value="",
        description="The path to the robot description file",
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_ros2_control_hardware_type_cmd)
    ld.add_action(declare_ros2_control_hardware_type_positon_joint_cmd)
    ld.add_action(declare_robot_description_path_cmd)

    ld.add_action(OpaqueFunction(function=launch_robot_state_publisher))

    return ld
