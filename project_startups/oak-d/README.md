# Oak-d Launch

This project setup is used to launch the oak-d driver for different use cases. Be aware that you might need to adjust the configurations in the respective `oak_d_config.yaml`.
## Default

To launch the Oak-d for the default use case, run (don't forget to build and source):
- `ros2 launch oak_bringup stereo_launch.py`

## Door Opening Mechanism

To launch the Oak-d for the door opening mechanism, run (don't forget to build and source):
- `ros2 launch oak_bringup_door_opening door_opening_cam_launch.py`

## Door Handle Detection

To launch the Oak-d for the door handle detection, run (don't forget to build and source):
- `ros2 launch door_handle_detection yolo.launch.py`

Note:
    Launch the door handle detection and then the door opening mechanism