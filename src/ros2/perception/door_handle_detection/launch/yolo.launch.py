import os
import yaml
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('door_handle_detection'), 'config', 'door_handle_detection_config.yaml')

    with open(config_file_path, 'r') as config_file:
        config_params = yaml.safe_load(config_file)

    def get_param(param_name):
        return LaunchConfiguration(param_name, default=config_params[param_name])

    door_handle_detection_path = get_package_share_directory('door_handle_detection')
    urdf_launch_dir = os.path.join(get_package_share_directory('oak_bringup_door_opening'), 'launch')
    resources_path = os.path.join(door_handle_detection_path, 'resources')

    params = {
        'mxId': get_param('mxId'),
        'usb2Mode': get_param('usb2Mode'),
        'poeMode': get_param('poeMode'),
        'camera_model': get_param('camera_model'),
        'tf_prefix': get_param('tf_prefix'),
        'mode': get_param('mode'),
        'base_frame': get_param('base_frame'),
        'parent_frame': get_param('parent_frame'),
        'cam_pos_x': get_param('cam_pos_x'),
        'cam_pos_y': get_param('cam_pos_y'),
        'cam_pos_z': get_param('cam_pos_z'),
        'cam_roll': get_param('cam_roll'),
        'cam_pitch': get_param('cam_pitch'),
        'cam_yaw': get_param('cam_yaw'),
        'lrcheck': get_param('lrcheck'),
        'extended': get_param('extended'),
        'subpixel': get_param('subpixel'),
        'rectify': get_param('rectify'),
        'depth_aligned': get_param('depth_aligned'),
        'manualExposure': get_param('manualExposure'),
        'expTime': get_param('expTime'),
        'sensIso': get_param('sensIso'),
        'syncNN': get_param('syncNN'),
        'detectionClassesCount': get_param('detectionClassesCount'),
        'nnName': get_param('nnName'),
        'resourceBaseFolder': resources_path,
        'stereo_fps': get_param('stereo_fps'),
        'confidence': get_param('confidence'),
        'LRchecktresh': get_param('LRchecktresh'),
        'monoResolution': get_param('monoResolution'),
        'rgbResolution': get_param('rgbResolution'),
        'rgbScaleNumerator': get_param('rgbScaleNumerator'),
        'rgbScaleDenominator': get_param('rgbScaleDenominator'),
        'previewWidth': get_param('previewWidth'),
        'previewHeight': get_param('previewHeight'),
        'enableRosBaseTimeUpdate': get_param('enableRosBaseTimeUpdate'),
        'robot_description_remapping': get_param('robot_description_remapping'),
        'color_image_topic': get_param('color_image_topic'),
        'door_handle_position_topic': get_param('door_handle_position_topic'),
        'stereo_depth_topic': get_param('stereo_depth_topic'),
        'right_rectified_image_topic': get_param('right_rectified_image_topic'),
        'coordinate_size': get_param('coordinate_size'),
        'depth_lower_threshold':get_param('depth_lower_threshold'),
        'depth_upper_threshold':get_param('depth_upper_threshold')
    }

    urdf_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(urdf_launch_dir, 'urdf_launch.py')
        ),
        launch_arguments={key: value for key, value in params.items() if key in {
            'tf_prefix', 'camera_model', 'base_frame', 'parent_frame',
            'cam_pos_x', 'cam_pos_y', 'cam_pos_z', 'cam_roll', 'cam_pitch',
            'cam_yaw', 'robot_description_remapping'
        }}.items()
    )

    door_handle_node = launch_ros.actions.Node(
        package='door_handle_detection', executable='yolov5_door_node',
        output='screen',
        parameters=[params]
    )

    ld = LaunchDescription()

    for param_name in config_params.keys():
        ld.add_action(DeclareLaunchArgument(param_name, default_value=str(config_params[param_name])))

    ld.add_action(urdf_launch)
    ld.add_action(door_handle_node)
    
    return ld
