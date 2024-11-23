import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():
    start_publish_pose_from_spatial_detection = launch_ros.actions.Node(
        package="publish_pose_from_spatial_detection",
        executable="publish_pose_from_spatial_detection",
        name="publish_pose_from_spatial_detection",
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(start_publish_pose_from_spatial_detection)

    return ld
