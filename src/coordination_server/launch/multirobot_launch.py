
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'turtlebot3_burger.urdf.xacro'

  print("urdf_file_name : {}".format(urdf_file_name))

  urdf = os.path.join(
      get_package_share_directory('ros2_sim_pkg'),
      urdf_file_name)

  return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            #namespace='robot1',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            #namespace='robot2',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
            ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "cam_bot"]
            ),
    
  ])

 #   robot_name = 'turtlebot3_burger'
  #  world_file_name = 'empty.world'
    
  #  world = os.path.join(get_package_share_directory(robot_name), 'worlds', world_file_name)

 #   urdf = os.path.join(get_package_share_directory(robot_name), 'sdf', 'turtlebot3_burger.sdf')

 #   xml = open(urdf, 'r').read()

  #  xml = xml.replace('"', '\\"')

 #   swpan_args = '{name: \"turtlebot3_burger\", xml: \"'  +  xml + '\" }'

 #   return LaunchDescription([
  #      ExecuteProcess(
  #          cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
  #          output='screen'),

   #     ExecuteProcess(
   #         cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
    #        output='screen'),

    #    ExecuteProcess(
    #        cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args],
    #        output='screen'),
   # ])'