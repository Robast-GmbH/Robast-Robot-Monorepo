from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
import launch_ros.actions

def generate_stereo_launch_description(default_values, urdf_launch_dir, namespace, remappings):
    camera_model = LaunchConfiguration('camera_model',  default = default_values['camera_model'])
    tf_prefix    = LaunchConfiguration('tf_prefix',       default = default_values['tf_prefix'])
    base_frame   = LaunchConfiguration('base_frame',    default = default_values['base_frame'])
    parent_frame = LaunchConfiguration('parent_frame',  default = default_values['parent_frame'])

    cam_pos_x  = LaunchConfiguration('cam_pos_x',     default = default_values['cam_pos_x'])
    cam_pos_y  = LaunchConfiguration('cam_pos_y',     default = default_values['cam_pos_y'])
    cam_pos_z  = LaunchConfiguration('cam_pos_z',     default = default_values['cam_pos_z'])
    cam_roll   = LaunchConfiguration('cam_roll',      default = default_values['cam_roll'])
    cam_pitch  = LaunchConfiguration('cam_pitch',     default = default_values['cam_pitch'])
    cam_yaw    = LaunchConfiguration('cam_yaw',       default = default_values['cam_yaw'])

    mode           = LaunchConfiguration('mode', default = default_values['mode'])
    lrcheck        = LaunchConfiguration('lrcheck', default = default_values['lrcheck'])
    extended       = LaunchConfiguration('extended', default = default_values['extended'])
    subpixel       = LaunchConfiguration('subpixel', default = default_values['subpixel'])
    confidence     = LaunchConfiguration('confidence', default = default_values['confidence'])
    LRchecktresh   = LaunchConfiguration('LRchecktresh', default = default_values['LRchecktresh'])
    monoResolution = LaunchConfiguration('monoResolution',  default = default_values['monoResolution'])

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='set to depth or disparity. Setting to depth will publish depth or else will publish disparity.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_monoResolution_cmd = DeclareLaunchArgument(
        'monoResolution',
        default_value=monoResolution,
        description='Contains the resolution of the Mono Cameras. Available resolutions are 800p, 720p & 400p for OAK-D & 480p for OAK-D-Lite.')

    urdf_launch = IncludeLaunchDescription(
                            urdf_launch_dir,
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
                                              'namespace'   : namespace}.items())

    stereo_node = launch_ros.actions.Node(
            package='depthai_examples',
            executable='stereo_node',
            output='screen',
            remappings=remappings,
            namespace=namespace,
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
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    namespace=namespace,
                    remappings=[('image_raw', '/stereo/depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('image', '/stereo/converted_depth')]
                ),
            ],
            output='screen',)

    point_cloud_node = launch_ros.actions.ComposableNodeContainer(
            name='container2',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',
                    namespace=namespace,
                    remappings=[('depth/image_rect', '/stereo/converted_depth'),
                                ('intensity/image_rect', '/right/image_rect'),
                                ('intensity/camera_info', '/right/camera_info'),
                                ('points', '/stereo/points')]
                ),
            ],
            output='screen',)

    return [
        declare_tf_prefix_cmd,
        declare_camera_model_cmd,
        declare_base_frame_cmd,
        declare_parent_frame_cmd,
        declare_pos_x_cmd,
        declare_pos_y_cmd,
        declare_pos_z_cmd,
        declare_roll_cmd,
        declare_pitch_cmd,
        declare_yaw_cmd,
        declare_mode_cmd,
        declare_lrcheck_cmd,
        declare_extended_cmd,
        declare_subpixel_cmd,
        declare_confidence_cmd,
        declare_LRchecktresh_cmd,
        declare_monoResolution_cmd,
        stereo_node,
        urdf_launch,
        metric_converter_node,
        point_cloud_node
    ]