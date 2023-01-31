from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    # joint parameter for rb_theron
    joint_names_list = ["drawer_1_joint",
                        "drawer_2_joint",
                        "drawer_3_joint",
                        "drawer_4_joint",
                        "drawer_5_joint",
                        "door_opening_mechanism_joint_y_axis_slide",
                        "door_opening_mechanism_joint_x_axis_slide",
                        "door_opening_mechanism_joint_tilting_hook"]
    gz_joint_topics_list = []
    for joint_name in joint_names_list:
        gz_joint_topics_list.append("/model/rb_theron/joint/%s/0/cmd_pos"%joint_name)
    # In case you want to test this manually use this command:
    # gz topic -t "/model/rb_theron/joint/door_opening_mechanism_joint_tilting_hook/0/cmd_pos" -m gz.msgs.Double -p "data: 0.1"

    follow_joint_trajectory_action = LaunchConfiguration("follow_joint_trajectory_action")
    
    follow_joint_trajectory_action_cmd = DeclareLaunchArgument(
        "follow_joint_trajectory_action",
        default_value="/door_opening_mechanism_controller/follow_joint_trajectory",
        description="Action on which the planned trajectory, which should be executed now, should be published.",
    )

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
    ld.add_action(joint_position_controller)
    ld.add_action(joint_trajectory_controller)
    return ld
