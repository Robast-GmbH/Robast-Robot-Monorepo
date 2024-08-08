# Setting up Moveit Pro

Check the confluence page for more informations: [Setup MoveIt Pro](https://robast.atlassian.net/wiki/spaces/ROBOTNIK/pages/132743181/MoveIt+Studio+MoveIt+Pro)

# How to run our Development Container with MoveIt Pro

Generally the process for this workflow is described here: [Create Overlay Docker Images](https://docs.picknik.ai/en/stable/getting_started/configuration_tutorials/docker_configuration/create_overlay_docker_image.html#create-overlay-docker-images)
But at the point of writing this, the guide contained a lot of errors and missing information.

In order to start this with the terminal, you have to enter the directory that contains our `docker-compose.yaml`, `Dockerfile` and the `.env` file first. This can be important for some commands because they might need the environment variables defined in the `.env` file.

First step is to build the development docker:

`docker compose build dev`

Afterwards run the `dev` service and the `web_ui` service of the docker compose by entering:

`docker compose up dev web_ui`

Then open 4 seperate terminals and enter the development container:

`docker compose exec -it dev bash`

Use one terminal to do a `colcon build`.

Then use the 4 terminals to run (you might need to `source install/setup.bash`):

`agent.app` or `ros2 launch rb_theron_config studio_agent_launch.py`

`robot.app` (not sure if this is always required)

`studio_bridge.app`

`rest_api.app`

If you want to run door handle detection open another terminal and run:

`ros2 launch door_handle_detector_sim door_handle_detection_sim.launch.py` (you might need to run `pip install yolov5` before and you might need `pip3 install setuptools==65.5.1` after that)

# How to run MoveIt Pro over ssh

In order to connect via ssh to a host maschine, where you want to run MoveIt Pro, you have to get access to the display. This can be done by setting the DISPLAY environment variable at the host system (so where MoveIt Pro runs) with `export DISPLAY=:0` or `export DISPLAY=:1` (in my case it was `:1`). Furthermore you need to connect with the argument `ssh -X`. When doing this with vs code you need to add `ForwardX11 yes` to your `config` file in the .ssh directory, so it looks something like this:
```
Host Jacob_Office_Desktop_PC
  HostName 10.10.23.9
  User jacob
  ForwardX11 yes
```
**Additional hint**: It might be necessary to log in at the target device. This was at least one observation from the past to get this to run, which might be due to my system setup where I have two displays connected to my target device and the OS properly "assigns" the displays not before loggin in?

