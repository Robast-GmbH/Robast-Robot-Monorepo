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

`agent.app`

`robot.app` (not sure if this is always required)

`studio_bridge.app`

`rest_api.app`
