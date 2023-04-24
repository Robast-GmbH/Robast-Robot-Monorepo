from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # joint parameter for rb_theron
    joint_names_default_list = [
        "drawer_1_joint",
        "drawer_2_joint",
        "drawer_3_joint",
        "drawer_4_joint",
        "drawer_5_joint",
    ]

    joint_names_list = LaunchConfiguration("joint_names_list")

    joint_names_cmd = DeclareLaunchArgument(
        "joint_names_list",
        default_value=joint_names_default_list,
        description="The list of joints that is used for the movement. Mind that for now the order of \
                    the joints is relevant and must match the order in the moveit_controller.",
    )

    #  ROS -> IGN,  joint position controller
    robot_trajectory_executor_node = Node(
        package="gazebo_controller_manager",
        executable="robot_trajectory_executor",
        name="rb_theron_robot_trajectory_executor",
        parameters=[
            {"joint_names": joint_names_list},
        ],
        output="screen",
    )

    ld.add_action(joint_names_cmd)
    ld.add_action(robot_trajectory_executor_node)
    return ld
