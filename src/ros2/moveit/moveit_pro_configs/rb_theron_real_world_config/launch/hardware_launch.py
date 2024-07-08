import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    common_launch_arguments = {
        "prefix": "/robot",
        "ros2_control_hardware_type": "dryve_d1",
        "ros2_control_hardware_type_positon_joint": "real_life",
        "position_joint_type": "prismatic",
        "model_door_opening_mechanism": "true",
        "model_module_cage": "false",
        "model_sensors": "false",
    }

    launch_arguments_ros2_control = {
        **common_launch_arguments,
        "robot_description_path": "config/rb_theron.urdf.xacro",
        "robot_description_semantic_file_path": "config/rb_theron_real_world.srdf",
        "trajectory_execution_file_path": "config/moveit_controllers_real_world.yaml",
        "sensor_3d_file_path": "config/sensors_3d_real_world.yaml",
        "joint_limits_file_path": "config/joint_limits_real_world.yaml",
        "ompl_planning_file": "ompl_iron",
    }

    launch_arguments_robot_state_publisher = {
        **common_launch_arguments,
        "use_sim_time": "false",
    }
    launch_arguments_robot_state_publisher["robot_description_path"] = os.path.join(
        get_package_share_directory("rb_theron_description"),
        "robots",
        "rb_theron_arm.urdf.xacro",
    ),

    # Start ros2 control controller
    launch_ros2_control_cmd =IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("moveit_door_opening_mechanism_config"),
        "launch",
        "ros2_control_real_world_launch.py")),
        launch_arguments=launch_arguments_ros2_control.items(),
    )

    # Start robot state publisher
    launch_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("moveit_door_opening_mechanism_config"),
            "launch",
            "robot_state_publisher_launch.py")),
        launch_arguments=launch_arguments_robot_state_publisher.items(),
    )

    ld = LaunchDescription()
    ld.add_action(launch_ros2_control_cmd)
    ld.add_action(launch_robot_state_publisher_cmd)

    return ld
