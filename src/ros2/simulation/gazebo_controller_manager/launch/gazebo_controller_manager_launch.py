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

    follow_joint_trajectory_action = LaunchConfiguration(
        "follow_joint_trajectory_action"
    )
    joint_names_list = LaunchConfiguration("joint_names_list")

    follow_joint_trajectory_action_cmd = DeclareLaunchArgument(
        "follow_joint_trajectory_action",
        default_value="/door_opening_mechanism_controller/follow_joint_trajectory",
        description="Action on which the planned trajectory, which should be executed now, \
                    should be published.",
    )
    joint_names_cmd = DeclareLaunchArgument(
        "joint_names_list",
        default_value=joint_names_default_list,
        description="The list of joints that is used for the movement. Mind that for now the order of \
                    the joints is relevant and must match the order in the moveit_controller.",
    )

    #  ROS -> IGN,  joint position controller
    # We can either use the joint position controller or the joint trajectory controller.
    # In case we want moveit to plan and execute the motion,
    # we have to go with the trajectory controller
    joint_position_controller = Node(
        package="gazebo_controller_manager",
        executable="joint_position_controller",
        name="rb_theron_joint_position_controller",
        parameters=[
            {"joint_names": joint_names_list},
        ],
        output="screen",
    )

    joint_trajectory_controller = Node(
        package="gazebo_controller_manager",
        executable="joint_trajectory_controller",
        name="rb_theron_joint_trajectory_controller",
        parameters=[
            {"joint_names": joint_names_list},
            {"rate": 200},
            {"follow_joint_trajectory_action": follow_joint_trajectory_action},
        ],
        output="screen",
    )

    ld.add_action(follow_joint_trajectory_action_cmd)
    ld.add_action(joint_names_cmd)
    ld.add_action(joint_position_controller)
    ld.add_action(joint_trajectory_controller)
    return ld
