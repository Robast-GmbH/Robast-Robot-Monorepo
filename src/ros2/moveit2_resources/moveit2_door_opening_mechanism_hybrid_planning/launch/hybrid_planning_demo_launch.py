import os
import sys

from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch import LaunchDescription

sys.path.append(os.path.dirname(__file__))
from hybrid_planning_common import (
    load_yaml,
)


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("rb_theron", package_name="moveit2_door_opening_mechanism_config").to_moveit_configs()

    # Demo node
    common_hybrid_planning_param = load_yaml(
        "moveit_hybrid_planning", "config/common_hybrid_planning_params.yaml"
    )
    demo_node = Node(
        package="moveit2_door_opening_mechanism_hybrid_planning",
        executable="hybrid_planning_demo_node",
        name="hybrid_planning_demo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            common_hybrid_planning_param,
            {"use_sim_time": True},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(demo_node)

    return ld
