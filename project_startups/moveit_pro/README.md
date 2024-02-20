# How to run MoveIt Pro

In order to start this with the terminal, you have to enter the directory that contains our `docker-compose.yaml`, `Dockerfile` and the `.env` file first. This can be important for some commands because they might need the environment variables defined in the `.env` file.

First step is to build the development docker:

`docker compose build dev`

Afterwards run the `dev` service and the `web_ui` service of the docker compose by entering:

`docker compose up dev web_ui`

Then open 4 seperate terminals and enter the development container:

`docker compose exec -it dev bash`

Then use the 4 terminals to run:

`agent.app`

`robot.app` (not sure if this is always required)

`studio.app`

`rest_api.app`
