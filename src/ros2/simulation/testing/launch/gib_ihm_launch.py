from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("testron", package_name="testing").to_moveit_configs()

    ld = LaunchDescription()

    door_opening_mechanism_simulation_node = Node(
        package="testing",
        executable="move_main",
        name="move_main",
        parameters=[
            {"use_sim_time": True},
            moveit_config.to_dict()
        ],
        output="screen",
    )

    ld.add_action(door_opening_mechanism_simulation_node)
    return ld
