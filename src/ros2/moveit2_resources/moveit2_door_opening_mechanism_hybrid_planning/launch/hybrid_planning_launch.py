import os
import sys

from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch import LaunchDescription

sys.path.append(os.path.dirname(__file__))
from hybrid_planning_common import (
    generate_common_hybrid_launch_description,
)


def generate_launch_description():

    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    common_launch = generate_common_hybrid_launch_description()

    ld = LaunchDescription()

    ld.add_action(common_launch)

    return ld
