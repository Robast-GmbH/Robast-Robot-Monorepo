#!/bin/bash

docker build "../../src/ros2/hardware_nodes/" -t hardware_nodes

docker run --name=hardware_nodes_socket --network=host -it --detach -v $(pwd)/../../src/ros2/hardware_nodes/drawer_bridge/:/workspace/src/hardware_nodes/drawer_bridge \
 -v $(pwd)/../../src/ros2/hardware_nodes/drawer_sym/:/workspace/src/hardware_nodes/drawer_sym \
 -v $(pwd)/../../src/ros2/hardware_nodes/drawer_manager/:/workspace/src/hardware_nodes/drawer_manager \
 -v $(pwd)/../../src/ros2/communication_interfaces/:/workspace/src/communication_interfaces \
 -v $(pwd)/../../libs/:/workspace/libs \
 hardware_nodes

bash setup_slcan.sh

docker exec hardware_nodes_socket /bin/bash -c "cd /workspace && apt-get update && apt-get upgrade -y && rosdep update && rosdep install --from-paths src --ignore-src -r -y && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build && source install/setup.bash && ros2 launch ros2_socketcan socket_can_bridge.launch.xml"
docker exec -d hardware_nodes_socket /bin/bash -c "cd /workspace; source install/setup.bash; ros2 launch drawer_bridge drawer_bridge_launch.py" 
