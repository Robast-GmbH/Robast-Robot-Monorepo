#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.legacy import LaunchTestService



def generate_launch_description():

    statemachine_bringup_dir = get_package_share_directory('drawer_sm')
    sim_bringup_dir = get_package_share_directory('tiplu_world')
    drawer_sim_dir = get_package_share_directory('drawer_bridge_simulation')
    contoller_manager_sim_dir = get_package_share_directory('gazebo_controller_manager')
    drawer_moveit_sim_dir = get_package_share_directory('moveit2_drawer_config')
    
    sim_arguments = {
        "headless": " -s",
        "world_model": os.path.join(get_package_share_directory("tiplu_world"), "worlds", "6OG" + ".sdf")
    }.items()
    
    contoller_manager_sim_arguments = {
        "follow_joint_trajectory_action": "/drawer_planning_group_controller/follow_joint_trajectory",
        "joint_names_list": "['drawer_1_joint','drawer_2_joint','drawer_3_joint','drawer_4_joint','drawer_5_joint']"
    }.items()
    
    moveit_drawer_sim_arguments = { "use_rviz": "False" }.items()
    
    ignition_tiplu_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bringup_dir, 'launch', 'tiplu_world_launch.py')),launch_arguments=sim_arguments)
    
    nfc_statemachine_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(statemachine_bringup_dir, 'launch', 'nfc_drawer_statemachine_launch.py')))
    
    
    drawer_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drawer_sim_dir, 'launch', 'drawer_bridge_simulation_launch.py')))
    
    contoller_manager_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(contoller_manager_sim_dir, 'launch', 'gazebo_controller_manager_launch.py')), 
                                      launch_arguments=contoller_manager_sim_arguments)
    
    drawer_moveit_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drawer_moveit_sim_dir, 'launch', 'moveit_launch.py')), 
                                      launch_arguments=moveit_drawer_sim_arguments)
    
    ld = LaunchDescription()
    
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))
    
    ld.add_action(ignition_tiplu_world_launch)
    ld.add_action(nfc_statemachine_launch)
    ld.add_action(drawer_sim_launch)
    ld.add_action(contoller_manager_sim_launch)
    ld.add_action(drawer_moveit_sim_launch)
    
    return ld

def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    nfc_test_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'test_nfc_drawer.py')],
        name='nfc_drawer_test_node',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, nfc_test_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())