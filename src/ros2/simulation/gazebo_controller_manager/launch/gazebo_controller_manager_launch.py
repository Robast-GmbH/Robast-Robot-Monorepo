import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    # joint parameter for rb_theron
    joint_names_default_list = "drawer_joints"
    
    follow_joint_trajectory_action = LaunchConfiguration("follow_joint_trajectory_action")
    joint_names = LaunchConfiguration("joint_names")
    
    follow_joint_trajectory_action_cmd = DeclareLaunchArgument(
        "follow_joint_trajectory_action",
        default_value = "/door_opening_mechanism_controller/follow_joint_trajectory",
        description = "Action on which the planned trajectory, which should be executed now, should be published.",
    )
    joint_names_cmd = DeclareLaunchArgument(
        "joint_names",
        default_value = joint_names_default_list,
        description = "Choose the list of joints that is used for the movement. The selection of lists you can pick from can be found under config/joint_names_list.yaml",
    )

    joint_names_yaml_path = os.path.join(
        get_package_share_directory("gazebo_controller_manager"),
        "config",
        "joint_names_list.yaml",
    )

    with open(joint_names_yaml_path, 'r') as stream:
        try:
            joint_names_yaml = yaml.safe_load(stream)
            print(joint_names_yaml)
        except yaml.YAMLError as exc:
            print(exc)

    joint_names_list = joint_names_yaml[joint_names]

    gz_joint_topics_list = []
    for joint_name in joint_names_list:
        print(joint_name) # DEBUGGING
        gz_joint_topics_list.append("/model/rb_theron/joint/%s/0/cmd_pos"%joint_name)
    # In case you want to test this manually use this command:
    # gz topic -t "/model/rb_theron/joint/door_opening_mechanism_joint_tilting_hook/0/cmd_pos" -m gz.msgs.Double -p "data: 0.1"

    #  ROS -> IGN,  joint position controller
    # We can either use the joint position controller or the joint trajectory controller.
    # In case we want moveit to plan and execute the motion, we have to go with the trajectory controller
    joint_position_controller = Node(
                package='gazebo_controller_manager',
                executable='joint_position_controller',
                name="rb_theron_joint_position_controller",
                parameters=[
                            {"joint_names": joint_names_list},
                            {"gz_joint_topics": gz_joint_topics_list},
                            {"rate": 200},
                           ],
                output='screen')
    
    joint_trajectory_controller = Node(
                package='gazebo_controller_manager',
                executable='joint_trajectory_controller',
                name="rb_theron_joint_trajectory_controller",
                parameters=[
                            {"joint_names": joint_names_list},
                            {"gz_joint_topics": gz_joint_topics_list},
                            {"rate": 200},
                            {"follow_joint_trajectory_action": follow_joint_trajectory_action}
                           ],
                output='screen')

    ld.add_action(follow_joint_trajectory_action_cmd)
    ld.add_action(joint_names_cmd)
    ld.add_action(joint_position_controller)
    ld.add_action(joint_trajectory_controller)
    return ld
