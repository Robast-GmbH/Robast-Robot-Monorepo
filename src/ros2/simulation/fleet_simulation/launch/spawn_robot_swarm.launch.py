import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import GroupAction


def generate_launch_description():

    ros_distro = os.environ["ROS_DISTRO"]

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="whether to use sim time or not",
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    for n in range(1):

        robot_name = "RB"+str(n)
        namespace= "/"+robot_name
        init_x = os.environ['init_x']
        init_y = str(0+10*n)
        init_yaw = os.environ["init_yaw"]

        robot_xml = xacro.process_file(
        os.path.join(
            get_package_share_directory("rb_theron_description"),
            "robots",
            os.environ["robot"] + ".urdf.xacro",
        ),
        mappings={"prefix": os.environ["prefix"],
                  "ros2_control_hardware_type": "gz_ros2_control",
                  "ros_distro": ros_distro,
                  "namespace":namespace}
        ).toxml()

        declare_robot_model_cmd = DeclareLaunchArgument(
            "robot_name",
            default_value="rb_theron",
            description="name of the robot in the simulation",
        
        )

        start_robot_state_publisher_cmd = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace= namespace,
            parameters=[{"use_sim_time": use_sim_time}, {"robot_description": robot_xml}],
            output="screen",
        )


        spawn_robot_cmd = Node(
            package="ros_gz_sim",
            executable="create",
            namespace= namespace,
            arguments=[
                "-name",
                robot_name,
                "-topic",
                namespace+"/robot_description",
                "-z",
                "0.2",
                "-x",
                init_x,
                "-y",
                init_y,
                "-Y",
                init_yaw,
            ],
            output="screen",
        )

        load_joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--controller-manager','/test/controller_manager', '--set-state', 'active', 'joint_state_broadcaster'],
            output='screen'
        )

        load_joint_trajectory_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--controller-manager','/test/controller_manager','--set-state', 'active', 'joint_trajectory_controller'],
            output='screen'
        )

        load_diff_drive_base_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller','--controller-manager','/test/controller_manager', '--set-state', 'active', 'diff_drive_base_controller'],
            output='screen'
        )

        load_drawer_joint_trajectory_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--controller-manager','/test/controller_manager', '--set-state', 'active', 'drawer_joint_trajectory_controller'],
            output='screen'
        )

        group = GroupAction([
            declare_robot_model_cmd,
            start_robot_state_publisher_cmd,
            spawn_robot_cmd,
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot_cmd,
                on_exit=[load_joint_state_broadcaster],
            )),
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )),
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_diff_drive_base_controller],
            )),
            RegisterEventHandler(
              event_handler=OnProcessExit(
                target_action=load_diff_drive_base_controller,
                on_exit=[load_drawer_joint_trajectory_controller],
            ))
        ])
        
        ld.add_action(group)
  
    # arguments
    ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_robot_model_cmd)
    
  


    # nodes
    # ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(spawn_robot_cmd)

    # ld.add_action(RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=spawn_robot_cmd,
    #             on_exit=[load_joint_state_broadcaster],
    #         )
    #     ))
    # ld.add_action(RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_joint_state_broadcaster,
    #             on_exit=[load_joint_trajectory_controller],
    #         )
    #     ))
    # ld.add_action(RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_joint_trajectory_controller,
    #             on_exit=[load_diff_drive_base_controller],
    #         )
    #     ))
    # ld.add_action(RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_diff_drive_base_controller,
    #             on_exit=[load_drawer_joint_trajectory_controller],
    #         )
    #     ))

    return ld
