import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    with open("/workspace/src/navigation/environment_vars.yaml", "r") as stream:
        try:
            environment_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    start_free_fleet_server_cmd = Node(
                package="free_fleet_server_ros2",
                executable="free_fleet_server_ros2",
                name="free_fleet_server",
                parameters=[{"fleet_name": "ROBAST_1"},
                            {"fleet_state_topic": "fleet_states"},
                            {"mode_request_topic": "robot_mode_requests"},
                            {"path_request_topic": "robot_path_requests"},
                            {"destination_request_topic": "robot_destination_requests"},

                            {"dds_domain": 42},
                            {"dds_robot_state_topic": "robot_state"},
                            {"dds_mode_request_topic": "mode_request"},
                            {"dds_path_request_topic": "path_request"},
                            {"dds_destination_request_topic": "destination_request"},

                            {"update_state_frequency": 20.0},
                            {"publish_state_frequency": 2.0},

                            {"translation_x": -4.117},
                            {"translation_y": 27.26},
                            {"rotation": -0.013},
                            {"scale": 0.928}],
                output="screen",
    )

    client_nodes = []

    for n in range(len(environment_yaml["robot_namspaces"])):
        robot_namespace = environment_yaml["robot_namspaces"][n]

        free_fleet_client = Node(
                package="free_fleet_client_ros2",
                executable="free_fleet_client_ros2",
                namespace=robot_namespace,
                name="free_fleet_client",
                parameters=[{"fleet_name": "ROBAST_1"},
                            {"robot_frame": robot_namespace+"/base_footprint"},
                            {"robot_name": robot_namespace},
                            {"robot_model": "rb_Theron"},
                            {"level_name": "L1"},
                            {"dds_domain": 42},
                            {"max_dist_to_first_waypoint": 10.0},
                            {"nav2_server_name": "/"+robot_namespace+"/navigate_to_pose"},
                            {"docking_trigger_server_name": "temp"}, ],
                output="screen",
         )
        client_nodes.append(free_fleet_client)

    ld = LaunchDescription()
    ld.add_action(start_free_fleet_server_cmd)
    for node in client_nodes:
        ld.add_action(node)

    return ld
