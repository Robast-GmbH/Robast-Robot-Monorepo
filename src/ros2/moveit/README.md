## MoveIt

This directory contains all packages that use MoveIt.

### moveit_resources

This package contains resources that are used by MoveIt like MoveIt configs or IK solver plugins.

If we want to execute a planned trajectory, there 3 different hardware interfaces we can do it with:

1. **mock_components** - This is used for executing the trajectory only in Rviz. Simply run `ros2 launch moveit_door_opening_mechanism_config moveit_rviz_simulation_launch.py` for that.
2. **gz_ros2_contro** - This is used for executing the trajectory in Gazebo. You have to first start Gazebo in the simulation
simulation docker (e.g. with `ros2 launch tiplu_world tiplu_world_launch.py`). Then you can run `ros2 launch moveit_door_opening_mechanism_config moveit_gz_simulation_launch.py` and plan a trajectory with MoveIt in
RViz, that will be executed in Gazebo.
1. **dryve_d1** - This is used for executing a trajectory in real life with our igus dryve d1 motor control. This can
be started if you are in the *moveit* project startup as we need the dryve_d1_bridge for that and the custom ros2_control
plugin. To launch this, run `ros2 launch moveit_door_opening_mechanism_config moveit_real_world_launch.py`


### moveit_mtc

This package contains code that uses the MoveIt Task Constructor Framework.

#### door_opening_mechanism_mtc

For now this is a simple demo, which attempts to open a door in real life (and already successfully did). This node listens for a 
pose of the door handle and then plans and executes a movement to open the door. To start this example
you need to run the following thing:
1. Start the `perception` docker and run `ros2 launch door_handle_detection yolo.launch.py` to get door handle detections.
2. Start the `moveit` docker and run `ros2 launch moveit_door_opening_mechanism_config moveit_real_world_launch.py` (runs ros2 control and moveit)
3. In the `moveit` docker run `ros2 launch door_opening_mechanism_mtc door_opening_mechanism_mtc_launch.py` (runs mtc)
4. Trigger the door opening by publishing `ros2 topic pub /trigger_door_opening std_msgs/msg/Empty --once`.


### moveit_pro_configs

This package contains the configurations need to launch our robot in MoveIt Pro. Please mind that these configurations can only be used with MoveIt Pro.