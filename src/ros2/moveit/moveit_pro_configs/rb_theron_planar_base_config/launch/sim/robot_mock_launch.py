# Copyright 2023 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    static_tf = Node(package='tf2_ros',
                         executable='static_transform_publisher',
                         name='static_transform_publisher',
                         output='log',
                         arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'odom'])

    # TODO(moveit_studio#7004): make mdof_joint_state_publisher a proper MoveGroup capability, or ros2_controller?
    odom_to_mdof_joint_state_repub = Node(package='stretch_re1_pro_config',
                                          executable='repub_odometry_mdof_joint_states.py',
                                          name='odom_to_mdof_joint_state_repub',
                                          output='log')

    planning_pipelines = ["ompl_iron"]

    rviz_config_file = (
        get_package_share_directory("moveit_door_opening_mechanism_config")
        + "/config/moveit_planar_base.rviz"
    )

    launch_arguments = {
        "ros2_control_hardware_type": "mock_components",
        "ros2_control_hardware_type_positon_joint": "real_life",
        "position_joint_type": "planar",
        "model_door_opening_mechanism": "true",
    }

    moveit_config = (
        MoveItConfigsBuilder(
            "rb_theron",
            package_name="moveit_door_opening_mechanism_config",
        )
        .robot_description(
            file_path="config/rb_theron.urdf.xacro", mappings=launch_arguments
        )
        .robot_description_semantic(file_path="config/rb_theron_planar_base.srdf")
        .trajectory_execution(file_path="config/moveit_simple_controller_manager_planar_base.yaml")
        .planning_pipelines(pipelines=planning_pipelines)
        .robot_description_kinematics(file_path="config/kinematics_planar_base.yaml")
        .to_moveit_configs()
    )

    # Rviz
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )
                           

    return LaunchDescription([static_tf, odom_to_mdof_joint_state_repub, rviz_node])
