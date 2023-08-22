import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration




def generate_launch_description():

        nav_launch_bringup_dir = os.path.join(get_package_share_directory("nav_bringup"), "launch")

        slam_toolbox_launch_file = LaunchConfiguration("slam_toolbox_launch_file")
        robot_localization_launch_file = LaunchConfiguration("robot_localization_launch_file")
        nav_launch_file = LaunchConfiguration("nav_launch_file")


        declare_slam_toolbox_launch_file = DeclareLaunchArgument(
                "slam_toolbox_launch_file",
                default_value=os.path.join(
                        nav_launch_bringup_dir,
                        "slam_toolbox_launch.py",
                ),
                description="path to the slam toolbox launch file",
        )
        
        declare_robot_localization_launch_file = DeclareLaunchArgument(
                "robot_localization_launch_file",
                default_value=os.path.join(
                        nav_launch_bringup_dir,
                        "robot_localization_odom_to_base_launch.py",
                ),
                description="path to the robot_localization launch file",
        )
        
        declare_nav_launch_file = DeclareLaunchArgument(
                "nav_launch_file",
                default_value=os.path.join(
                        nav_launch_bringup_dir,
                        "nav_without_localization_launch.py",
                ),
                description="path to the nav2 launch file",
        )

        slam_toolbox_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch_file)
        )
        
        robot_localization_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_localization_launch_file)
        )
        
        nav_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch_file)
        )

        ld = LaunchDescription()
        ld.add_action(declare_slam_toolbox_launch_file)
        ld.add_action(declare_robot_localization_launch_file)
        ld.add_action(declare_nav_launch_file)
        
        ld.add_action(robot_localization_launch)
        ld.add_action(slam_toolbox_launch)
        ld.add_action(nav_launch)
        
        return ld