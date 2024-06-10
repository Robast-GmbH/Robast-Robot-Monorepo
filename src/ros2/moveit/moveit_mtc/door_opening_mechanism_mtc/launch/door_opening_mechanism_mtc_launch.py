import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_arguments = {
        "ros2_control_hardware_type": "gz_ros2_control",
        "model_position_joint": "prismatic",
    }

    ros_distro = os.environ["ROS_DISTRO"]
    if ros_distro == "humble":
        ompl_planning_pipeline = "ompl_humble"
        planning_pipelines = [ompl_planning_pipeline]
    else:
        ompl_planning_pipeline = "ompl_iron"
        planning_pipelines = [ompl_planning_pipeline]

    moveit_config = (
        MoveItConfigsBuilder(
            "rb_theron",
            package_name="moveit_door_opening_mechanism_config",
        )
        .robot_description(
            file_path="config/rb_theron.urdf.xacro", mappings=launch_arguments
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    door_opening_mechanism_mtc_node = Node(
        package="door_opening_mechanism_mtc",
        executable="door_opening_mechanism_mtc",
        name="door_opening_mechanism_mtc",
        parameters=[
            {"moveit2_planning_group_name": "mobile_base_arm"},
            {"use_sim_time": True},
            {"planning_pipeline": ompl_planning_pipeline},
            moveit_config.to_dict(),
        ],
        output="screen",
    )

    ld.add_action(door_opening_mechanism_mtc_node)
    return ld
