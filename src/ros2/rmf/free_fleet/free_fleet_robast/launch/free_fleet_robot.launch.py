import launch
from launch_ros.actions import Node  
import yaml
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

def generate_launch_description():

    with open("/workspace/src/rmf/rmf_robast/config/fleet_vars.yaml", "r") as stream:
        try:
            fleet_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    robot_namespace = LaunchConfiguration('robot_namespace')
    robot_frame = LaunchConfiguration('robot_frame')  
    nav_server = LaunchConfiguration('nav_server')     

    robot_namespace_cmd = launch.actions.DeclareLaunchArgument(
        'robot_namespace',
        default_value='RB0',
        description='the namespace in which the robot nodes should be started in')
     
    nav_server_cmd = launch.actions.DeclareLaunchArgument(
        'nav_server',
        default_value='/navigate_to_pose',
        description='The Topic of nav for the navigation to a pose  ')
    
    robot_frame_cmd = launch.actions.DeclareLaunchArgument(
        'robot_frame',
        default_value="/base_footprint",
        description='the Base frame name of the Robot')
    
    free_fleet_client_cmd = Node(
        package="free_fleet_client_ros2",
        executable="free_fleet_client_ros2",
        namespace=robot_namespace,
        name="free_fleet_client",
        parameters=[{"fleet_name": fleet_yaml["fleet_name"]},
                    {"robot_frame": robot_frame},
                    {"robot_name": robot_namespace},
                    {"robot_model": "rb_Theron"},
                    {"level_name": "L1"},
                    {"dds_domain": 42},
                    {"max_dist_to_first_waypoint": 10.0},
                    {"nav2_server_name":nav_server},
                    {"docking_trigger_server_name": "temp"}, ],
                output="screen",
         )
    free_fleet_direct_client_cmd = Node(
          package="free_fleet_client_direct",
                executable="client_direct",
                namespace= launch.substitutions.LaunchConfiguration('robot_namespace'),
                name="free_fleet_client",
                )

    ld = LaunchDescription() 
    ld.add_action(robot_namespace_cmd)
    ld.add_action(nav_server_cmd)
    ld.add_action(robot_frame_cmd)

    ld.add_action(free_fleet_client_cmd)
    ld.add_action(free_fleet_direct_client_cmd)

    return ld