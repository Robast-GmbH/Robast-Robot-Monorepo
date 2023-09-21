from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch.conditions import IfCondition
from launch_ros.actions import Node



def generate_launch_description():

        moveit_config = MoveItConfigsBuilder("rb_theron", package_name="moveit_door_opening_mechanism_rotating_arm_config").to_moveit_configs()

        ld = LaunchDescription()
        ld.add_action(
                DeclareBooleanLaunchArg(
                "db",
                default_value=False,
                description="By default, we do not start a database (it can be large)",
                )
        )
        ld.add_action(DeclareBooleanLaunchArg(
                "use_rviz",
                description='Whether to start RViz',
                default_value=True))
                        
        ld.add_action(
                DeclareBooleanLaunchArg(
                "use_sim_time",
                default_value=True,
                description="Whether to use sim time or not"))

        ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
        ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))

        # load non-default MoveGroup capabilities (space separated)
        ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
        # inhibit these default MoveGroup capabilities (space separated)
        ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value="move_group/MoveGroupExecuteTrajectoryAction"))

        # do not copy dynamics information from /joint_states to internal robot monitoring
        # default to false, because almost nothing in move_group relies on this information
        ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

        should_publish = LaunchConfiguration("publish_monitored_planning_scene")
        use_sim_time = LaunchConfiguration("use_sim_time")
        use_rviz = LaunchConfiguration("use_rviz")
        monitor_dynamics = LaunchConfiguration("monitor_dynamics")
        allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")

        move_group_configuration = {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": allow_trajectory_execution,
                # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
                "capabilities": ParameterValue(
                LaunchConfiguration("capabilities"), value_type=str
                ),
                "disable_capabilities": ParameterValue(
                LaunchConfiguration("disable_capabilities"), value_type=str
                ),
                # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
                "publish_planning_scene": should_publish,
                "publish_geometry_updates": should_publish,
                "publish_state_updates": should_publish,
                "publish_transforms_updates": should_publish,
                "monitor_dynamics": monitor_dynamics,
                "use_sim_time": use_sim_time,
                "octomap_resolution": 0.05,
        }

        move_group_node = Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                        moveit_config.to_dict(),
                        move_group_configuration,
                ],
        )

        rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                respawn=False,
                arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
                parameters=[
                        moveit_config.planning_pipelines,
                        moveit_config.robot_description_kinematics,
                        {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(
                        PythonExpression(
                                [use_rviz]
                        )
                )
        )

        ld.add_action(rviz_node)
        ld.add_action(move_group_node)

        return ld
