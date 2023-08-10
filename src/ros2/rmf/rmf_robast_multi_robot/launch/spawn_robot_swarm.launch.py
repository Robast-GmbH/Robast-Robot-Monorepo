import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction


def generate_launch_description():

    with open("/workspace/src/rmf/rmf_robast/config/sim_config.yaml", "r") as stream:
        try:
            sim_config = yaml.safe_load(stream)
            print(sim_config)
        except yaml.YAMLError as exc:
            print(exc)

    ld = LaunchDescription()
    use_sim_time = True
    for n in range(len(sim_config["robot_namspaces"])):
        robot_namespace = sim_config["robot_namspaces"][n]
        robot_xml = xacro.process_file(
            os.path.join(
                get_package_share_directory("rb_theron_description"),
                "robots",
                sim_config["robot"]+".urdf.xacro",
            ),
            mappings={"prefix": sim_config["prefix"], "topic_namespace": robot_namespace},
        ).toxml()

        declare_robot_model_cmd = DeclareLaunchArgument(
            "robot_name",
            default_value=robot_namespace,
            description="name of the robot in the simulation",
        )

        start_robot_state_publisher_cmd = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=robot_namespace,
                parameters=[{'frame_prefix': robot_namespace+'/',
                             "use_sim_time": use_sim_time},
                            {"robot_description": robot_xml}],
                output="screen",
        )

        spawn_robot_cmd = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            namespace=robot_namespace,
            arguments=["-name",
                       robot_namespace,
                       "-topic",
                       "robot_description",
                       "-z",
                       "0.2",
                       "-x",
                       str(-2+-2*n),
                       "-y",
                       "0",
                       "-Y",
                       "0",
                       ],
        )

        group = GroupAction([
            declare_robot_model_cmd,
            start_robot_state_publisher_cmd,
            spawn_robot_cmd,
        ])
        ld.add_action(group)
    return ld
