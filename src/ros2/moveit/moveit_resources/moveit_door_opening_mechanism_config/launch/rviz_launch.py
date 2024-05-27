import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder

urdf_launch_arguments = {
    "ros2_control_hardware_type": "dryve_d1",
    "ros2_control_hardware_type_positon_joint": "real_life",
    "model_position_joint": "true",
}


def get_urdf_launch_arguments(context):

    urdf_launch_arguments["ros2_control_hardware_type"] = str(
        LaunchConfiguration("ros2_control_hardware_type").perform(context)
    )
    urdf_launch_arguments["ros2_control_hardware_type_positon_joint"] = str(
        LaunchConfiguration("ros2_control_hardware_type_positon_joint").perform(context)
    )
    urdf_launch_arguments["model_position_joint"] = str(
        LaunchConfiguration("model_position_joint").perform(context)
    )


def generate_launch_description():
    ld = LaunchDescription()

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

    declare_model_position_joint_cmd = DeclareLaunchArgument(
        "model_position_joint",
        default_value="true",
        description="Whether to use position joint (between base_link and base_footprint) or not",
    )

    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "humble":
        planning_pipelines = ["ompl_humble"]
    elif ros_distro == "iron":
        planning_pipelines = ["ompl_iron"]
    else:
        raise Exception("Unknown ROS distro: " + ros_distro)

    get_urdf_launch_arguments_opaque_func = OpaqueFunction(
        function=get_urdf_launch_arguments,
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "rb_theron",
            package_name="moveit_door_opening_mechanism_config",
        )
        .robot_description_semantic(file_path="config/rb_theron_arm.srdf")
        .robot_description(
            file_path="config/rb_theron_arm.urdf.xacro",
            mappings=urdf_launch_arguments,
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .to_moveit_configs()
    )

    rviz_config_file = (
        get_package_share_directory("moveit_door_opening_mechanism_config")
        + "/config/moveit.rviz"
    )
    rviz2_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    ld.add_action(declare_ros2_control_hardware_type_cmd)
    ld.add_action(declare_ros2_control_hardware_type_positon_joint_cmd)
    ld.add_action(declare_model_position_joint_cmd)

    ld.add_action(get_urdf_launch_arguments_opaque_func)

    ld.add_action(rviz2_cmd)

    return ld
