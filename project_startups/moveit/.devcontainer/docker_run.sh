# this script is used to run the docker container on the robot to have a rviz gui
# Please mind that you have to run this via Remmina
docker run \
	--name=moveit_devel \
	--network=host \
	-it \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix \
	--volume=/home/robot/robast/Monorepo/project_startups/moveit/:/workspace/ \
	--volume=/home/robot/robast/Monorepo/src/ros2/hardware_nodes/dryve_d1_bridge:/workspace/src/hardware_nodes/dryve_d1_bridge \
	--volume=/home/robot/robast/Monorepo/src/ros2/robot_description:/workspace/src/robot_description \
	--volume=/home/robot/robast/Monorepo/src/ros2/ros2_control/hardware_interfaces/ros2_control_plugin_dryve_d1:/workspace/src/ros2_control/hardware_interfaces/ros2_control_plugin_dryve_d1 \
	--volume=/home/robot/robast/Monorepo/src/ros2/ros2_control/hardware_interfaces/ros2_control_base_movement:/workspace/src/ros2_control/hardware_interfaces/ros2_control_base_movement \
	--volume=/home/robot/robast/Monorepo/src/ros2/ros2_control/hardware_interfaces/hardware_interface_utils:/workspace/src/ros2_control/hardware_interfaces/hardware_interface_utils \
	--volume=/home/robot/robast/Monorepo/src/ros2/moveit:/workspace/src/moveit \
	--volume=/home/robot/robast/Monorepo/src/ros2/ros2_control/ros2_controllers:/workspace/src/ros2_control/ros2_controllers \
	--volume=/home/robot/robast/Monorepo/src/ros2/dds_configs:/workspace/dds_configs \
	--volume=/home/robot/robast/Monorepo/src/ros2/utils/launch_manager:/workspace/src/utils/launch_manager \
	-e DISPLAY=:0 \
	--env ROS_DOMAIN_ID=23 \
	--env robot=rb_theron \
	--env prefix="" \
	--user robast \
	ghcr.io/robast-gmbh/monorepo/moveit:devel-humble \
	bash -c "cd ~/../../workspace && sudo apt-get update && sudo apt-get upgrade -y && rosdep update && rosdep install --from-paths src --ignore-src -r -y && colcon build --packages-skip get_spatial_detections_from_topic create_pose_from_spatial_detections publish_pose_to_topic setup_mtc_move_to_pose_with_constraints && bash"

