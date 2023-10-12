## MoveIt

This directory contains all packages that use MoveIt.

### moveit_resources

This package contains resources that are used by MoveIt like MoveIt configs or IK solver plugins.

### moveit_mtc

This package contains code that uses the MoveIt Task Constructor Framework.

#### door_opening_mechanism_mtc

For now this is a simple demo to get a first running example with using mtc for our robot. This node listens for a 
pose of the door handle and then plans a movement to this position and back to a starting pose. To start this example
you need to start the tasks:
1. `ros2 launch moveit_door_opening_mechanism_rotating_arm_config moveit_launch.py` (runs rviz and MoveIt)
2. `ros2 launch door_opening_mechanism_mtc door_opening_mechanism_mtc_launch.py` (runs mtc)
3. `ros2 launch door_opening_mechanism_simulation door_opening_mechanism_simulation_send_fake_pose_launch.py` (sends a door handle pose to the mtc node to trigger mtc)