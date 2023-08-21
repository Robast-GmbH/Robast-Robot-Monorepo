import yaml
import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():

    # load configs
    with open("/workspace/src/rmf/rmf_robast/config/fleet_vars.yaml", "r") as stream:
        try:
            fleet_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    ros2_rmf_fleet_states = LaunchConfiguration('rmf_fleet_states_topic')
    ros2_rmf_mode_request = LaunchConfiguration('rmf_mode_request_topic')
    ros2_rmf_path_request = LaunchConfiguration('rmf_path_request_topic')
    ros2_rmf_destination_request = LaunchConfiguration('rmf_destination_request_topic')

    ros2_rmf_fleet_states_topic_cmd = launch.actions.DeclareLaunchArgument(
        'rmf_fleet_states_topic',
        default_value=fleet_yaml["ros2_rmf_fleet_states_topic"],
        description='rmf_topic on which the robots publish there status location, tasks,.. ')

    ros2_rmf_mode_request_topic_cmd = launch.actions.DeclareLaunchArgument(
        'rmf_mode_request_topic',
        default_value=fleet_yaml["ros2_rmf_mode_request_topic"],
        description='rmf_topic on which the mode of operation of the robot is sent')

    ros2_rmf_path_request_topic_cmd = launch.actions.DeclareLaunchArgument(
        'rmf_path_request_topic',
        default_value=fleet_yaml["ros2_rmf_path_request_topic"],
        description='rmf_topic on which the waypoint the robot should move throgh is sent')

    ros2_rmf_destination_request_topic_cmd = launch.actions.DeclareLaunchArgument(
        'rmf_destination_request_topic',
        default_value=fleet_yaml["ros2_rmf_destination_request_topic"],
        description='rmf_topic on which the final target position of the Robot is sent')

    # start the main free fleet server
    start_free_fleet_server_cmd = Node(
                package="free_fleet_server_ros2",
                executable="free_fleet_server_ros2",
                name="free_fleet_server",
                parameters=[{"fleet_name": fleet_yaml["fleet_name"]},
                            {"fleet_state_topic": ros2_rmf_fleet_states},
                            {"mode_request_topic": ros2_rmf_mode_request},
                            {"path_request_topic": ros2_rmf_path_request},
                            {"destination_request_topic": ros2_rmf_destination_request},

                            {"dds_domain": fleet_yaml["dds_domain"]},
                            {"dds_robot_state_topic": fleet_yaml["dds_robot_state_topic"]},
                            {"dds_mode_request_topic": fleet_yaml["dds_mode_request_topic"]},
                            {"dds_path_request_topic": fleet_yaml["dds_path_request_topic"]},
                            {"dds_destination_request_topic":
                             fleet_yaml["dds_destination_request_topic"]},

                            {"update_state_frequency": fleet_yaml["update_state_frequency"]},
                            {"publish_state_frequency": fleet_yaml["publish_state_frequency"]},

                            {"translation_x": fleet_yaml["translation_x"]},
                            {"translation_y": fleet_yaml["translation_y"]},
                            {"rotation": fleet_yaml["rotation"]},
                            {"scale": fleet_yaml["scale"]}],
                output="screen",
    )

    # start a server component to provide a custom rest interface for rmf
    # and to extend the functionality with custom features.
    start_api_server_cmd = Node(
            package="api_connector",
            executable="web_api",
            name="web_controller",
            )

    ld = LaunchDescription()

    ld.add_action(ros2_rmf_mode_request_topic_cmd)
    ld.add_action(ros2_rmf_fleet_states_topic_cmd)
    ld.add_action(ros2_rmf_path_request_topic_cmd)
    ld.add_action(ros2_rmf_destination_request_topic_cmd)

    ld.add_action(start_free_fleet_server_cmd)
    ld.add_action(start_api_server_cmd)
    return ld
