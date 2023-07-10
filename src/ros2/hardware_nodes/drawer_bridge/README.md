# Drawer Bridge

The drawer bridge connects the drawer_sm with the drawers by using a ros2_socketcan node.

Communication diagram:
    drawer_sm <--ros_topics--> drawer_bridge <--ros_topics--> ros2_socketcan <--can_interface--> drawer_controller

Launch commands:
    ros2 launch drawer_bridge drawer_bridge_launch.py
    ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:='vxcan1'

(socketcan interface may differ)

Error handle: 03