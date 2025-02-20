## Idea behind the Monorepo

The idea behind the Monorepo is to consolidate all the code, configurations, and documentation for various projects into a single repository. This approach simplifies dependency management, ensures consistency across projects, and facilitates easier collaboration among team members. By having a unified repository, we can leverage shared libraries and tools, streamline the development workflow, and maintain a coherent versioning strategy. The Monorepo structure also allows for better visibility into the overall project architecture and enables more efficient CI/CD processes.

## Repository Structure

This repository is organized into several directories and files, each serving a specific purpose. Below is an overview of the main components:

- `.vscode/`: Contains Visual Studio Code settings.
- `.github/`: Contains GitHub-specific files.
  - `dependabot.yml`: Configuration for Dependabot.
  - `pull_request_template.md`: Template for pull requests.
  - `workflows/`: Contains GitHub Actions workflows for CI/CD.
- `bringups/`: Facilitates the setup and configuration of various components for **production** use.
  - `debugging_docker/`: Debugging configurations for Docker.
  - `docker_healthchecks/`: Health check scripts for Docker containers.
  - `hardware_nodes/`: Scripts and configurations for hardware nodes.
  - `jetson/`: Configurations and start scripts for Jetson devices.
  - `Robo_A/`: Configurations and start scripts for Robo A.
  - `Robo_B/`: Configurations and start scripts for Robo B.
  - `rviz/`: Configurations for RViz.
- `project_startups/`: Contains startup configurations for various projects which is mainly used for **development**.
- `src/`: Contains the source code for various components. Here most of the projects should have its own Dockerfile.
  - `Dockerfile`: Dockerfile for building the base image.
  - `Hardware/`: Contains hardware-related source code and libraries.
    - `cpp/`: C++ libraries and source code.
    - `platformio_libs/`: PlatformIO libraries for embedded development.
    - `py/`: Python libraries and source code.
  - `preprocessing/`: Contains preprocessing scripts and libraries.
  - `robot_backend/`: Contains the backend code for the robot.
  - `ros2/`: Contains ROS 2 related source code.

## Using Docker Containers

Docker containers are used extensively in this repository to ensure a consistent and reproducible development environment. By containerizing our applications and services, we can easily manage dependencies, streamline the build process, and simplify deployment. Below are some key points on how we use Docker containers:

### Building Docker Images

We use GitHub Actions workflows to automate the process of building Docker images. Each project typically has its own Dockerfile located in the `src/` directory. The workflows are defined in the `.github/workflows/` directory and include steps to build and push Docker images to the GitHub Container Registry.

For example, the workflow for building the `statemachine` project is defined in [`.github/workflows/docker_create_statemachine.yml`](.github/workflows/docker_create_statemachine.yml). It includes steps to:

- Checkout the repository and submodules.
- Login to the GitHub Container Registry.
- Configure Docker Buildx.
- Build and push the Docker image for the `statemachine` project.

### Running Docker Containers

Docker containers are used to run tests, generate coverage reports, and execute various tasks. The workflows include steps to run Docker containers with the necessary volumes and environment variables.

For example, the workflow for running tests on the `statemachine` project is defined in [`.github/workflows/run_statemachine_tests.yml`](.github/workflows/run_statemachine_tests.yml). It includes steps to:

- Checkout the repository and submodules.
- Login to the GitHub Container Registry.
- Run a Docker container with the necessary volumes and environment variables.
- Execute tests and generate coverage reports inside the container.

### Managing Docker Cache

To speed up the build process, we use Docker layer caching. The workflows include steps to cache Docker layers using the `actions/cache` action. This helps to avoid rebuilding layers that have not changed, reducing build times.

For example, the workflow for building the `statemachine` project includes steps to:

- Cache Docker layers before building the image.
- Restore the cache during subsequent builds to speed up the process.

### Multi-Platform Builds

We support multi-platform builds to ensure our Docker images can run on different architectures, such as `amd64` (needed for the robot base) and `arm64` (needed on the jetson devices). The workflows include steps to build and push images for multiple platforms using Docker Buildx.

For example, the workflow for building the `robast-source` image is defined in [`.github/workflows/docker_create_ros2_src.yml`](.github/workflows/docker_create_ros2_src.yml). It includes steps to:

- Configure Docker Buildx for multi-platform builds.
- Build and push the `robast-source` image for `linux/amd64` and `linux/arm64/v8` platforms.




