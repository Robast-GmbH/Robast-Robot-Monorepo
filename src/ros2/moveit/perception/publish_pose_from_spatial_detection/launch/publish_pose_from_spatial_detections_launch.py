import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_confidence_threshold = LaunchConfiguration("use_confidence_threshold")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    invert_y_axis = LaunchConfiguration("invert_y_axis")
    target_orientation_in_euler = LaunchConfiguration("target_orientation_in_euler")
    target_frame_pose_stamped = LaunchConfiguration("target_frame_pose_stamped")
    topic_name_spatial_detections = LaunchConfiguration("topic_name_spatial_detections")
    topic_name_pose_stamped = LaunchConfiguration("topic_name_pose_stamped")

    declare_use_confidence_threshold_cmd = DeclareLaunchArgument(
        "use_confidence_threshold",
        default_value="False",
        description="Use confidence threshold to filter spatial detections",
    )

    declare_confidence_threshold_cmd = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.5",
        description="Confidence threshold to filter spatial detections",
    )

    declare_invert_y_axis_cmd = DeclareLaunchArgument(
        "invert_y_axis", default_value="False", description="Invert y axis of the pose"
    )

    declare_target_orientation_in_euler_cmd = DeclareLaunchArgument(
        "target_orientation_in_euler",
        default_value="[0.0, 1.57079632679, 0.0]",
        description="Target orientation in euler angles",
    )

    declare_target_frame_pose_stamped_cmd = DeclareLaunchArgument(
        "target_frame_pose_stamped",
        default_value="robot/base_footprint",
        description="Target frame of the pose stamped",
    )

    declare_topic_name_spatial_detections_cmd = DeclareLaunchArgument(
        "topic_name_spatial_detections",
        default_value="/stereo/door_handle_position",
        description="Topic name of spatial detections",
    )

    declare_topic_name_pose_stamped_cmd = DeclareLaunchArgument(
        "topic_name_pose_stamped",
        default_value="/stereo/door_handle_pose",
        description="Topic name of pose stamped",
    )

    start_publish_pose_from_spatial_detection = launch_ros.actions.Node(
        package="publish_pose_from_spatial_detection",
        executable="publish_pose_from_spatial_detection",
        name="publish_pose_from_spatial_detection",
        output="screen",
        parameters=[
            {"use_confidence_threshold": use_confidence_threshold},
            {"confidence_threshold": confidence_threshold},
            {"invert_y_axis": invert_y_axis},
            {"target_orientation_in_euler": target_orientation_in_euler},
            {"target_frame_pose_stamped": target_frame_pose_stamped},
            {"topic_name_spatial_detections": topic_name_spatial_detections},
            {"topic_name_pose_stamped": topic_name_pose_stamped},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_confidence_threshold_cmd)
    ld.add_action(declare_confidence_threshold_cmd)
    ld.add_action(declare_invert_y_axis_cmd)
    ld.add_action(declare_target_orientation_in_euler_cmd)
    ld.add_action(declare_target_frame_pose_stamped_cmd)
    ld.add_action(declare_topic_name_spatial_detections_cmd)
    ld.add_action(declare_topic_name_pose_stamped_cmd)
    ld.add_action(start_publish_pose_from_spatial_detection)

    return ld
