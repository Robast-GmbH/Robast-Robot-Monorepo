This folder contains the implementation of Open-RMF which enables our fleet to manage it's traffic (including door opening/closing) and assign tasks to the fleets robots

To run the fleet_management:

- Start Simulation (optional)
- Start Navigation
- Start nav_action_node and robot_pose_publisher in nav container
  - `ros2 launch nav_action_node nav_action_node.launch.py`
  - `ros2 launch robot_pose_publisher robot_pose_publisher.launch.py` (if sim -> use_robot_source_frame:=False)
- Start Robot Backend in robot_backend container for reallife else included in simulation container
    - `cd src/robot_backend`
    - `pip install -r requirements.txt`
    - `bash start_server_with_rosbridge.sh`
- Start Middleware in middleware container
    - `cd middleware`
    - `python3 main.py`
- Start RMF API-Server in fleet_management container
    - `cd ../rmf-web-workspace/rmf-web`
    - `pnpm install`
    - `cd packages/dashboard`
    - `pnpm start`
- Start open_rmf nodes in fleet_management container
    - `colcon build --symlink-install`
    - `source install/setup.bash`
    - `ros2 run dispenser_ingestor_mock dispenser_ingestor_mock`
    - if not sim (since doors aren't implemented yet for the sim) -> `ros2 run door_adapter_template door_adapter_template`
    - `ros2 launch fleet_adapter_rb_theron rmf_launch.py`


    

