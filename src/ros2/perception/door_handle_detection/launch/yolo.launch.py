import os
import launch_ros.actions
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    door_handle_detection_path = get_package_share_directory('door_handle_detection')
    urdf_launch_dir = os.path.join(get_package_share_directory('oak_bringup_door_opening'), 'launch')
    resources_path = os.path.join(door_handle_detection_path, 'resources')

    mxId         = LaunchConfiguration('mxId',      default = 'x')
    usb2Mode     = LaunchConfiguration('usb2Mode',  default = False)
    poeMode      = LaunchConfiguration('poeMode',   default = False)
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')
    mode         = LaunchConfiguration('mode', default = 'depth')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'robot/door_opening_mechanism_link_y_axis_slide')
    cam_pos_x    = LaunchConfiguration('cam_pos_x',     default = '-0.05')
    cam_pos_y    = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z    = LaunchConfiguration('cam_pos_z',     default = '0.13721')
    cam_roll     = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch    = LaunchConfiguration('cam_pitch',     default = '0.5235988')
    cam_yaw      = LaunchConfiguration('cam_yaw',       default = '3.14159')
    lrcheck        = LaunchConfiguration('lrcheck', default = True)
    extended       = LaunchConfiguration('extended', default = False)
    subpixel       = LaunchConfiguration('subpixel', default = False)
    rectify        = LaunchConfiguration('rectify', default = True)
    depth_aligned  = LaunchConfiguration('depth_aligned', default = True)
    manualExposure = LaunchConfiguration('manualExposure', default = False)
    expTime        = LaunchConfiguration('expTime', default = 20000)
    sensIso        = LaunchConfiguration('sensIso', default = 800)
    syncNN                  = LaunchConfiguration('syncNN', default = True)
    detectionClassesCount   = LaunchConfiguration('detectionClassesCount', default = 2)
    nnName                  = LaunchConfiguration('nnName', default = 'yolov5.blob')
    resource_base_folder      = LaunchConfiguration('resourceBaseFolder', default = resources_path)
    stereo_fps            = LaunchConfiguration('stereo_fps', default = 30)
    confidence            = LaunchConfiguration('confidence', default = 120)
    LRchecktresh          = LaunchConfiguration('LRchecktresh', default = 5)
    monoResolution        = LaunchConfiguration('monoResolution', default = '400p')
    rgbResolution           = LaunchConfiguration('rgbResolution', default = '1080p')
    rgbScaleNumerator       = LaunchConfiguration('rgbScaleNumerator', default = 2)
    rgbScaleDenominator     = LaunchConfiguration('rgbScaleDenominator', default = 3)
    preview_width            = LaunchConfiguration('previewWidth', default = 480)
    previewHeight           = LaunchConfiguration('previewHeight', default = 640)
    enableRosBaseTimeUpdate       = LaunchConfiguration('enableRosBaseTimeUpdate', default = False)

    robot_description_remapping = LaunchConfiguration('robot_description_remapping', default = 'oak/robot_description')
    

    declare_mxId_cmd = DeclareLaunchArgument(
        'mxId',
        default_value=mxId,
        description='select the device by passing the MxID of the device. It will connect to first available device if left empty.')
    
    declare_usb2Mode_cmd = DeclareLaunchArgument(
        'usb2Mode',
        default_value=usb2Mode,
        description='To revert and use usb2 Mode. Set this parameter to false')
    
    declare_poeMode_cmd = DeclareLaunchArgument(
        'poeMode',
        default_value=poeMode,
        description='When MxID is set and the device is a POE model then set the poeMode to \"true\" to connect properly.')
    
    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')
    
    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='your custom name for the prefix of camera TF frames')
 
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='set to  \"depth\" or \"disparity\". Setting to depth will publish depth or else will publish disparity.')
    
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link in the TF Tree.')
    
    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from an another robot TF that can be connected to the base of the OAK device.')

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
    
    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='LR-Check is used to remove incorrectly calculated disparity pixels due to occlusions at object borders. Set to true to enable it')
    
    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='Extended disparity mode allows detecting closer distance objects for the given baseline. Set this parameter to true to enable it')
    
    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='Subpixel mode improves the precision and is especially useful for long range measurements. It also helps for better estimating surface normals. Set this parameter to true to enable it')
    
    declare_rectify_cmd = DeclareLaunchArgument(
        'rectify',
        default_value=rectify,
        description='enable this to publish rectified images used for depth estimation')
    
    declare_depth_aligned_cmd = DeclareLaunchArgument(
        'depth_aligned',
        default_value=depth_aligned,
        description='When depth_aligned is enabled depth map from stereo will be aligned to the RGB camera in the center.')
    
    declare_manualExposure_cmd = DeclareLaunchArgument(
        'manualExposure',
        default_value=manualExposure,
        description='When manualExposure is enabled, you can set the exposure time(expTime) and ISO(sensIso) of the stereo camera.')
    
    declare_expTime_cmd = DeclareLaunchArgument(
        'expTime',
        default_value=expTime,
        description='Set the exposure time of the stereo camera. Default value is 20000')
    
    declare_sensIso_cmd = DeclareLaunchArgument(
        'sensIso',
        default_value=sensIso,
        description='Set the ISO of the stereo camera. Default value is 800')
    
    declare_syncNN_cmd = DeclareLaunchArgument(
        'syncNN',
        default_value=syncNN,
        description='When syncNN is enabled Preview Image will be synced with the Detections.')
    
    declare_detectionClassesCount_cmd = DeclareLaunchArgument(
        'detectionClassesCount',
        default_value=detectionClassesCount,
        description='When detectionClassesCount is number of classes the NN contains. Default is set to 2.')

    declare_nnName_cmd = DeclareLaunchArgument(
        'nnName',
        default_value=nnName,
        description='Name of the NN blob being used to load. By default the one in resources folder will be used.')
    
    declare_resourceBaseFolder_cmd = DeclareLaunchArgument(
        'resourceBaseFolder',
        default_value=resource_base_folder,
        description='Path to the folder where NN Blob is stored.')
    
    declare_stereo_fps_cmd = DeclareLaunchArgument(
        'stereo_fps',
        default_value=stereo_fps,
        description='Sets the FPS of the cameras used in the stereo setup.')
    
    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='Set the confidence of the depth from 0-255. Max value means allow depth of all confidence. Default is set to 200')
    
    declare_rgbScaleNumerator_cmd = DeclareLaunchArgument(
        'rgbScaleNumerator',
        default_value=rgbScaleNumerator,
        description='Number of the scale Factor Numerator on top of RGB resolution selection.')
    
    declare_rgbScaleDenominator_cmd = DeclareLaunchArgument(
        'rgbScaleDenominator',
        default_value=rgbScaleDenominator,
        description='Number of the scale Factor Dinominator on top of RGB resolution selection.')
    
    declare_rgbResolution_cmd = DeclareLaunchArgument(
        'declare_rgbResolution_cmd',
        default_value=rgbResolution,
        description='Set the resolution of the RGB setup. Choose between 1080p, 4k, 12MP.')
    
    declare_monoResolution_cmd = DeclareLaunchArgument(
        'declare_monoResolution_cmd',
        default_value=monoResolution,
        description='Set the resolution of the mono/Stereo setup. Choose between 720p, 400p, 480p, 800p.')
    
    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'declare_LRchecktresh_cmd',
        default_value=LRchecktresh,
        description='Set the LR threshold from 1-10 to get more accurate depth. Default value is 5.')
    
    declare_previewWidth_cmd = DeclareLaunchArgument(
        'previewWidth',
        default_value=preview_width,
        description='Set the width of the preview window used for the NN detection.')
    
    declare_previewHeight_cmd = DeclareLaunchArgument(
        'previewHeight',
        default_value=previewHeight,
        description='Set the height of the preview window used for the NN detection.')

    declare_enableRosBaseTimeUpdate_cmd = DeclareLaunchArgument(
        'enableRosBaseTimeUpdate',
        default_value=enableRosBaseTimeUpdate,
        description='Whether to update ROS time on each message.')
    
    declare_robot_description_remapping_cmd = DeclareLaunchArgument(
        'robot_description_remapping',
        default_value='oak/robot_description',
        description='Remap the robot_description topic to another topic. Default value will be `oak/robot_description`')

    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'tf_prefix'   : tf_prefix,
                                              'camera_model': camera_model,
                                              'base_frame'  : base_frame,
                                              'parent_frame': parent_frame,
                                              'cam_pos_x'   : cam_pos_x,
                                              'cam_pos_y'   : cam_pos_y,
                                              'cam_pos_z'   : cam_pos_z,
                                              'cam_roll'    : cam_roll,
                                              'cam_pitch'   : cam_pitch,
                                              'cam_yaw'     : cam_yaw,
                                              'robot_description_remapping': robot_description_remapping}.items())

    door_handle_node = launch_ros.actions.Node(
            package='door_handle_detection', executable='yolov5_door_node',
            output='screen',
            parameters=[{'mxId':                    mxId},
                        {'usb2Mode':                usb2Mode},
                        {'poeMode':                 poeMode},
                        {'resourceBaseFolder':      resource_base_folder},
                        {'tf_prefix':               tf_prefix},
                        {'mode':                    mode},
                        {'lrcheck':                 lrcheck},
                        {'extended':                extended},
                        {'subpixel':                subpixel},
                        {'rectify':                 rectify},
                        {'depth_aligned':           depth_aligned},
                        {'manualExposure':          manualExposure},
                        {'expTime':                 expTime},
                        {'sensIso':                 sensIso},
                        {'stereo_fps':              stereo_fps},
                        {'confidence':              confidence},
                        {'LRchecktresh':            LRchecktresh},
                        {'monoResolution':          monoResolution},
                        {'rgbResolution':           rgbResolution},
                        {'rgbScaleNumerator':       rgbScaleNumerator},
                        {'rgbScaleDenominator':     rgbScaleDenominator},
                        {'previewWidth':            preview_width},
                        {'previewHeight':           previewHeight},
                        {'detectionClassesCount':   detectionClassesCount},
                        {'syncNN':                  syncNN},
                        {'nnName':                  nnName},
                        {'enableRosBaseTimeUpdate': enableRosBaseTimeUpdate},
                        {'color_image_topic':'color/image'},
                        {'door_handle_position_topic': 'stereo/door_handle_position'},
                        {'stereo_depth_topic': 'stereo/depth'},
                        {'right_rectified_image_topic': 'right/image_rect'}])

    ld = LaunchDescription()
    ld.add_action(declare_mxId_cmd)
    ld.add_action(declare_usb2Mode_cmd)
    ld.add_action(declare_poeMode_cmd)
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_rectify_cmd)
    ld.add_action(declare_depth_aligned_cmd)
    ld.add_action(declare_manualExposure_cmd)
    ld.add_action(declare_expTime_cmd)
    ld.add_action(declare_sensIso_cmd)
    ld.add_action(declare_syncNN_cmd)
    ld.add_action(declare_detectionClassesCount_cmd)
    ld.add_action(declare_nnName_cmd)
    ld.add_action(declare_resourceBaseFolder_cmd)
    ld.add_action(declare_stereo_fps_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)
    ld.add_action(declare_monoResolution_cmd)
    ld.add_action(declare_rgbResolution_cmd)
    ld.add_action(declare_rgbScaleNumerator_cmd)
    ld.add_action(declare_rgbScaleDenominator_cmd)
    ld.add_action(declare_previewWidth_cmd)
    ld.add_action(declare_previewHeight_cmd)
    ld.add_action(declare_enableRosBaseTimeUpdate_cmd)
    ld.add_action(declare_robot_description_remapping_cmd)

    ld.add_action(urdf_launch)
    ld.add_action(door_handle_node)
    
    return ld