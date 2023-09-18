from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("rb_theron", package_name="moveit_door_opening_mechanism_rotating_arm_config").to_moveit_configs()

    ld = LaunchDescription()

    door_opening_mechanism_mtc_node = Node(
        package="door_opening_mechanism_mtc",
        executable="door_opening_mechanism_mtc",
        name="door_opening_mechanism_mtc",
        parameters=[
            {"moveit2_planning_group_name": "mobile_base_arm"}, # mobile_base_arm or door_opening_mechanism
            {"use_sim_time": True},
            moveit_config.to_dict()
        ],
        output="screen",
    )

    ld.add_action(door_opening_mechanism_mtc_node)
    return ld
