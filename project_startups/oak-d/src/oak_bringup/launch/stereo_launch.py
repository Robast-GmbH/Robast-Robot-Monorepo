import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration

sys.path.append(os.path.join(get_package_share_directory('oak_bringup'), 'launch'))
from utils.oak_launch_arguments import add_launch_arguments


def generate_launch_description():
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')

    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix  = LaunchConfiguration('tf_prefix',   default = 'oak_s')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'robot_base_link')

    cam_pos_x  = LaunchConfiguration('cam_pos_x',     default = '0.317')
    cam_pos_y  = LaunchConfiguration('cam_pos_y',     default = '-0.197')
    cam_pos_z  = LaunchConfiguration('cam_pos_z',     default = '1.3044')
    cam_roll   = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch  = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw    = LaunchConfiguration('cam_yaw',       default = '0.0')

    mode           = LaunchConfiguration('mode', default = 'depth')
    lrcheck        = LaunchConfiguration('lrcheck', default = True)
    extended       = LaunchConfiguration('extended', default = False)
    subpixel       = LaunchConfiguration('subpixel', default = True)
    confidence     = LaunchConfiguration('confidence', default = 120)
    LRchecktresh   = LaunchConfiguration('LRchecktresh', default = 5)
    monoResolution = LaunchConfiguration('monoResolution',  default = '720p')

    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'tf_prefix' : tf_prefix,
                                              'camera_model': camera_model,
                                              'base_frame'  : base_frame,
                                              'parent_frame': parent_frame,
                                              'cam_pos_x'   : cam_pos_x,
                                              'cam_pos_y'   : cam_pos_y,
                                              'cam_pos_z'   : cam_pos_z,
                                              'cam_roll'    : cam_roll,
                                              'cam_pitch'   : cam_pitch,
                                              'cam_yaw'     : cam_yaw}.items())

    stereo_node = launch_ros.actions.Node(
            package='depthai_examples', executable='stereo_node',
            output='screen',
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
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    remappings=[('image_raw', '/stereo/depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('image', '/stereo/converted_depth')]
                ),
            ],
            output='screen',)

    point_cloud_node = launch_ros.actions.ComposableNodeContainer(
            name='container2',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',

                    remappings=[('depth/image_rect', '/stereo/converted_depth'),
                                ('intensity/image_rect', '/right/image_rect'),
                                ('intensity/camera_info', '/right/camera_info'),
                                ('points', '/stereo/points')]
                ),
            ],
            output='screen',)

    ld = LaunchDescription()

    add_launch_arguments(ld)

    ld.add_action(stereo_node)
    ld.add_action(urdf_launch)

    ld.add_action(metric_converter_node)
    ld.add_action(point_cloud_node)
    return ld
