import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions

sys.path.append(os.path.join(get_package_share_directory('oak_bringup'), 'launch'))
from utils.oak_launch_arguments import *

def generate_launch_description():
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')

    TF_TOPIC = "/tf"
    TF_STATIC_TOPIC = "/tf_static"
    NAMESPACE_ARM = "arm"
    remappings_tf = [
        (TF_TOPIC, "/" + NAMESPACE_ARM + TF_TOPIC),
        (TF_STATIC_TOPIC, "/" + NAMESPACE_ARM + TF_STATIC_TOPIC),
    ]

    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix  = LaunchConfiguration('tf_prefix',       default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'back_top_oak_d_camera')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'door_opening_mechanism_link_y_axis_slide')

    cam_pos_x  = LaunchConfiguration('cam_pos_x',     default = '-0.05')
    cam_pos_y  = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z  = LaunchConfiguration('cam_pos_z',     default = '0.13721')
    cam_roll   = LaunchConfiguration('cam_roll',      default = '0')
    cam_pitch  = LaunchConfiguration('cam_pitch',     default = '0.5235988')
    cam_yaw    = LaunchConfiguration('cam_yaw',       default = '3.14159')

    mode           = LaunchConfiguration('mode', default = 'depth')
    lrcheck        = LaunchConfiguration('lrcheck', default = True)
    extended       = LaunchConfiguration('extended', default = True)
    subpixel       = LaunchConfiguration('subpixel', default = False)
    confidence     = LaunchConfiguration('confidence', default = 120)
    LRchecktresh   = LaunchConfiguration('LRchecktresh', default = 5)
    monoResolution = LaunchConfiguration('monoResolution',  default = '400p')

    urdf_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(os.path.join(get_package_share_directory('oak_bringup'), 'launch'), 'door_opening_cam_urdf_launch.py')),
                launch_arguments={'tf_prefix' : tf_prefix,
                                    'camera_model': camera_model,
                                    'base_frame'  : base_frame,
                                    'parent_frame': parent_frame,
                                    'cam_pos_x'   : cam_pos_x,
                                    'cam_pos_y'   : cam_pos_y,
                                    'cam_pos_z'   : cam_pos_z,
                                    'cam_roll'    : cam_roll,
                                    'cam_pitch'   : cam_pitch,
                                    'cam_yaw'     : cam_yaw,
                                    'namespace'   : NAMESPACE_ARM}.items())

    stereo_node = launch_ros.actions.Node(
            package='depthai_examples', executable='stereo_node',
            output='screen',
            remappings=remappings_tf,
            namespace=NAMESPACE_ARM,
            parameters=[{'tf_prefix': tf_prefix},
                        {'mode': mode},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel},
                        {'confidence': confidence},
                        {'LRchecktresh': LRchecktresh},
                        {'monoResolution': monoResolution}])


    metric_converter_node = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace=NAMESPACE_ARM,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    namespace=NAMESPACE_ARM,
                    remappings=[('image_raw', 'stereo/depth'),
                                ('camera_info', 'stereo/camera_info'),
                                ('image', 'stereo/converted_depth')]
                ),
            ],
            output='screen',)

    point_cloud_node = launch_ros.actions.ComposableNodeContainer(
            name='container2',
            namespace=NAMESPACE_ARM,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',
                    namespace=NAMESPACE_ARM,
                    remappings=[('depth/image_rect', 'stereo/converted_depth'),
                                ('intensity/image_rect', 'right/image_rect'),
                                ('intensity/camera_info', 'right/camera_info'),
                                ('points', 'stereo/points')]
                ),
            ],
            output='screen',)

    ld = LaunchDescription()
    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_camera_model_cmd)
    
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)
    
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)
    ld.add_action(declare_monoResolution_cmd)

    ld.add_action(stereo_node)
    ld.add_action(urdf_launch)

    ld.add_action(metric_converter_node)
    ld.add_action(point_cloud_node)
    return ld
