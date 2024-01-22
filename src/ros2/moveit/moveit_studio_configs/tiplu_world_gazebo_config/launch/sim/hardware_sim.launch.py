# Copyright 2023 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os
import re

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node

from moveit_studio_utils_py.launch_common import get_launch_file, get_ros_path
from moveit_studio_utils_py.system_config import get_config_folder, SystemConfigParser


def path_pattern_change_for_gazebo(urdf_string):
    # Replaces strings in a URDF file such as
    #     package://package_name/path/to/file
    # to the actual full path of the file.
    data = urdf_string
    package_expressions = re.findall("(package://([^//]*))", data)
    for expr in set(package_expressions):
        data = data.replace(expr[0], get_ros_path(expr[1]))
    return data


def generate_simulation_description(context, *args, **settings):
    use_gui = settings.get("gazebo_gui", False)
    is_verbose = settings.get("gazebo_verbose", False)

    world_model_package = settings.get("gazebo_world_model_package", "tiplu_world")
    world_name = settings.get("gazebo_world_name", "6OG.sdf")

    # Create a Gazebo world file that swaps out package:// paths with absolute path.
    world_file = os.path.join(
        get_package_share_directory(world_model_package),
        "worlds",
        world_name,
    )
    modified_world_file = os.path.join(
        get_config_folder(), "auto_created", "gazebo_world.sdf"
    )
    with open(world_file, "r") as file:
        world_sdf = path_pattern_change_for_gazebo(file.read())
    with open(modified_world_file, "w") as file:
        file.write(world_sdf)

    # Launch Gazebo.
    print(f"Starting Gazebo with world {world_name}")
    print(f"GUI: {use_gui}, Verbose: {is_verbose}")

    sim_args = "-r"
    if is_verbose:
        sim_args += " -v 4"
    if not use_gui:
        sim_args += " -s --headless-rendering"

    gazebo = IncludeLaunchDescription(
        get_launch_file("ros_gz_sim", "launch/gz_sim.launch.py"),
        launch_arguments=[("gz_args", [f"{sim_args} {modified_world_file}"])],
    )
    return [gazebo]


def generate_launch_description():
    system_config_parser = SystemConfigParser()
    optional_feature_setting = system_config_parser.get_optional_feature_configs()

    gz_ros_bridge_yaml = os.path.join(
        get_package_share_directory("tiplu_world"), "config", "ign_ros_bridge.yaml"
    )

    # Launch Gazebo
    gazebo = OpaqueFunction(
        function=generate_simulation_description, kwargs=optional_feature_setting
    )

    spawn_robot_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "rb_theron",
            "-topic",
            "robot_description",
            "-z",
            "0.2",
            "-x",
            "-2.0",
            "-y",
            "0.0",
            "-Y",
            "3.14",
        ],
        output="screen",
    )

    gz_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": gz_ros_bridge_yaml},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            spawn_robot_cmd,
            gz_ros_bridge_cmd,
        ]
    )
