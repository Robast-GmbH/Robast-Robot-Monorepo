# Perception

This directory contains all packages that are only required when running machine learning(ML) based tasks in the simulation or on the robot.

## door_handle_detection

This package contains the code for door handle detection on the robot via the OAK-D.
Start with task `start door_handle_detection` or via ros2 cmd (after sourcing setup file): `ros2 launch door_handle_detection yolo.launch.py`

## oak_bringup_door_opening

This package contains the code for generating pointcloud from the topics published by the OAK-D camera.
Start with task `start oak_for_pointcloud` or via ros2 cmd (after sourcing setup file): `ros2 launch oak_bringup_door_opening oak_stereo_points.launch.py`

## door_handle_detection_sim

This package contains the code for door handle detection in a gazebo simulation.


### Important before using the packages

The weights(binary files) required for the above mentioned ML tasks are stored in the git large file storage. The files in git large storage won’t entirely download onto the local branch by just a ´git pull’ on the branch. It will be present on the local branch just as a text pointer. To access the complete file:

    1. First install git lfs by running the following in your terminal:

        curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

        sudo apt-get install git-lfs 

    2. Type ´git lfs´ in the terminal and check if git lfs is recognized as a command.

    3. To access files from the large file storage run ´git lfs pull´ on the branch from the terminal, this would download the entire file onto your   
       local branch. 