#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    nav_bringup_dir = get_package_share_directory('nav_bringup')
    sim_bringup_dir = get_package_share_directory('tiplu_world')
    params_file = os.path.join(nav_bringup_dir, 'config_simulation/nav_params/nav2_params_humble.yaml')

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites='',
        convert_types=True)

    context = LaunchContext()
    new_yaml = configured_params.perform(context)
    
    sim_arguments = {
        "headless": " -s ",
        "world_model": os.path.join(get_package_share_directory("tiplu_world"), "worlds", "6OG" + ".sdf")
    }.items()
    
    ignition_tiplu_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bringup_dir, 'launch', 'tiplu_world_launch.py')),launch_arguments=sim_arguments)
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_bringup_dir, 'launch', 'nav_without_localization_launch.py')),
        launch_arguments={
                            'use_sim_time': 'True',
                            'use_composition': 'False',
                            'params_file': configured_params,
                            'autostart': 'True'}.items())
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_bringup_dir, 'launch', 'slam_toolbox_launch.py'))) #TODO @Tobi use sim time?
    
    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_bringup_dir, 'launch', 'robot_localization_odom_to_base_launch.py')))
    #rviz_launch?
    
    ld = LaunchDescription()
    
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))
    
    ld.add_action(ignition_tiplu_world_launch)
    ld.add_action(robot_localization_launch)
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)
    
    return ld

def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'tester.py')],
        name='tester_node',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())