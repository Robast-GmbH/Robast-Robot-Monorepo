import launch
import os
import sys

from launch_ros.actions import Node
from launch import LaunchDescription

sys.path.append(os.path.dirname(__file__))
from hybrid_planning_common import (
    generate_common_hybrid_launch_description,
    get_robot_description,
    get_robot_description_semantic,
    load_yaml,
)


def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    common_launch = generate_common_hybrid_launch_description()

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
            get_robot_description(),
            get_robot_description_semantic(),
            common_hybrid_planning_param,
            {"use_sim_time": True},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(common_launch)
    ld.add_action(demo_node)

    return ld
