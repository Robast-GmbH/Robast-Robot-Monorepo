import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    launch_arguments = {
        "ros2_control_hardware_type": "gz_ros2_control",
        "model_position_joint": "prismatic",
        "model_door_opening_mechanism": "true",
    }

    ros_distro = os.environ["ROS_DISTRO"]
    supported_ros_distros = ["humble", "iron"]
    if ros_distro in supported_ros_distros:
        planning_pipelines =  [f"ompl_{ros_distro}"]
    else:
        raise Exception("Unknown ROS distro: " + ros_distro)

    moveit_config = (
        MoveItConfigsBuilder(
            "rb_theron",
            package_name="moveit_door_opening_mechanism_config",
        )
        .robot_description(
            file_path="config/rb_theron.urdf.xacro", mappings=launch_arguments
        )
        .robot_description_semantic(file_path="config/rb_theron.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="whether to use sim time or not",
    )
    ld.add_action(declare_use_sim_time_cmd)


    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_door_opening_mechanism_config")
        + "/config/moveit_gazebo.rviz"
    )
    ld.add_action(
        Node(
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
                {"use_sim_time": use_sim_time},
            ],
        )
    )

    ld.add_action(Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'odom']))

    return ld
