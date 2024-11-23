import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    urdf_launch_arguments = {
        "ros2_control_hardware_type": "gz_ros2_control",
        "position_joint_type": "prismatic",
        "model_door_opening_mechanism": "true",
        "ros2_control_hardware_type": "mock_components",
        "ros2_control_hardware_type_positon_joint": "mock_components",
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
            file_path="config/rb_theron.urdf.xacro", mappings=urdf_launch_arguments
        )
        .robot_description_semantic(file_path="config/rb_theron_real_world.srdf")
        .trajectory_execution(file_path="config/moveit_controllers_real_world.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .joint_limits(file_path="config/joint_limits_real_world.yaml")
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
