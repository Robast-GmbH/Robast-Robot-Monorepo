import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import GroupAction
from launch import LaunchDescription

def generate_launch_description():

    ros_distro = os.environ["ROS_DISTRO"]

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    gz_version = os.environ["GZ_VERSION"]

    if (gz_version == "fortress"):
        gz_version_tag ="ignition"
            
    if (gz_version == "garden" or gz_version == "harmonic"):
        gz_version_tag ="gz"
    

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="whether to use sim time or not",
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    robot_topics=[] 
    for n in range(1):
        robot_name = "rb"+str(n)
        namespace= "/"+robot_name
        init_x = os.environ['init_x']
        init_y = str(-3+1*n)
        init_yaw = os.environ["init_yaw"]
        robot_xml = xacro.process_file(
        os.path.join(
            get_package_share_directory("rb_theron_description"),
            "robots",
            os.environ["robot"] + ".urdf.xacro",
        ),
        mappings={"prefix": os.environ["prefix"],
                  "ros2_control_hardware_type": "gz_ros2_control",
                  "ros_distro": ros_distro,
                  "robot_name":robot_name,
                  "namespace":namespace,
                  "use_gazebo_sensor_plugins": str(n==1)
                  }
        ).toxml()

        declare_robot_model_cmd = DeclareLaunchArgument(
            "robot_name",
            default_value=robot_name,
            description="name of the robot in the simulation",
        
        )

        start_robot_state_publisher_cmd = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace= namespace,
            parameters=[{"use_sim_time": use_sim_time}, {"robot_description": robot_xml}],
            output="screen",
        )


        spawn_robot_cmd = Node(
            package="ros_gz_sim",
            executable="create",
            namespace= namespace,
            arguments=[
                "-name",
                robot_name,
                "-topic",
                namespace+"/robot_description",
                "-z",
                "0.2",
                "-x",
                init_x,
                "-y",
                init_y,
                "-Y",
                init_yaw,
            ],
            output="screen",
        )

        load_joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--controller-manager',f"/{robot_name}/controller_manager", '--set-state', 'active', 'joint_state_broadcaster'],
            output='screen'
        )

        load_joint_trajectory_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--controller-manager',f"/{robot_name}/controller_manager",'--set-state', 'active', 'joint_trajectory_controller'],
            output='screen'
        )

        load_diff_drive_base_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller','--controller-manager',f"/{robot_name}/controller_manager", '--set-state', 'active', 'diff_drive_base_controller'],
            output='screen'
        )

        load_drawer_joint_trajectory_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--controller-manager',f"/{robot_name}/controller_manager", '--set-state', 'active', 'drawer_joint_trajectory_controller'],
            output='screen'
        )

        robot_topics.append(f"/{robot_name}/front_laser/scan@sensor_msgs/msg/LaserScan@{gz_version_tag}.msgs.LaserScan")
        robot_topics.append(f"/{robot_name}/bpearl_laser/scan/points@sensor_msgs/msg/PointCloud2@{gz_version_tag}.msgs.PointCloudPacked")
        
        robot_topics.append(f"/{robot_name}/front_realsense_camera/color@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/front_realsense_camera/depth@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/front_realsense_camera/depth/points@sensor_msgs/msg/PointCloud2@{gz_version_tag}.msgs.PointCloudPacked")
        
        robot_topics.append(f"/{robot_name}/back_top_oak_d_camera/color@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/back_top_oak_d_camera/depth@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/back_top_oak_d_camera/depth/points@sensor_msgs/msg/PointCloud2@{gz_version_tag}.msgs.PointCloudPacked")
        robot_topics.append(f"/{robot_name}/back_top_oak_d_camera/color_camera_info@sensor_msgs/msg/CameraInfo@{gz_version_tag}.msgs.CameraInfo")

        robot_topics.append(f"/{robot_name}/front_top_oak_d_camera/color@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/front_top_oak_d_camera/depth@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/front_top_oak_d_camera/depth/points@sensor_msgs/msg/PointCloud2@{gz_version_tag}.msgs.PointCloudPacked")
        robot_topics.append(f"/{robot_name}/front_top_oak_d_camera/color_camera_info@sensor_msgs/msg/CameraInfo@{gz_version_tag}.msgs.CameraInfo")

        robot_topics.append(f"/{robot_name}/back_top_realsense_camera/color@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/back_top_realsense_camera/depth@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/back_top_realsense_camera/depth/points@sensor_msgs/msg/PointCloud2@{gz_version_tag}.msgs.PointCloudPacked")

        robot_topics.append(f"/{robot_name}/back_realsense_camera/color@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/back_realsense_camera/depth@sensor_msgs/msg/Image@{gz_version_tag}.msgs.Image")
        robot_topics.append(f"/{robot_name}/back_realsense_camera/depth/points@sensor_msgs/msg/PointCloud2@{gz_version_tag}.msgs.PointCloudPacked")
        robot_topics.append(f"/{robot_name}/imu/data@sensor_msgs/msg/Imu@{gz_version_tag}.msgs.IMU")

        group = GroupAction([
            declare_robot_model_cmd,
            start_robot_state_publisher_cmd,
            spawn_robot_cmd,
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot_cmd,
                on_exit=[load_joint_state_broadcaster],
            )),
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )),
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_diff_drive_base_controller],
            )),
            RegisterEventHandler(
              event_handler=OnProcessExit(
                target_action=load_diff_drive_base_controller,
                on_exit=[load_drawer_joint_trajectory_controller],
            ))
        ])
        
        ld.add_action(group)

        gz_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments= robot_topics,
        output="screen",
    )

    # arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(gz_ros_bridge_cmd)

    return ld
