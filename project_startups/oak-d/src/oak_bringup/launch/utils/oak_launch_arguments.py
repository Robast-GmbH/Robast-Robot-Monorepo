from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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