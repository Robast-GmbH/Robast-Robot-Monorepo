# Docker image for extending MoveIt Pro with a custom overlay.
#
# Example build command (with defaults):
#
# docker build -f ./Dockerfile .
#

# Specify the MoveIt Pro release to build on top of.
ARG MOVEIT_STUDIO_BASE_IMAGE
ARG USERNAME=studio-user
ARG USER_UID=1000
ARG USER_GID=1000

##################################################
# Starting from the specified MoveIt Pro release #
##################################################
# The image tag is specified in the argument itself.
# hadolint ignore=DL3006
FROM ${MOVEIT_STUDIO_BASE_IMAGE} as base

# Create a non-root user
ARG USERNAME
ARG USER_UID
ARG USER_GID

# Pass some custom paths to this dockerfile
ARG ROBAST_SOURCE_DIR
ARG ROBAST_PROJECT_STARTUP_RELATIVE_DIR

# Copy source code from the workspace's ROS 2 packages to a workspace inside the container
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}
RUN mkdir -p ${USER_WS}/src ${USER_WS}/build ${USER_WS}/install ${USER_WS}/log
COPY ${ROBAST_SOURCE_DIR} ${USER_WS}/src

# Also mkdir with user permission directories which will be mounted later to avoid docker creating them as root
WORKDIR $USER_WS
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    groupadd --gid $USER_GID ${USERNAME} && \
    useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash --create-home ${USERNAME} && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    cp -r /etc/skel/. /home/${USERNAME} && \
    mkdir -p \
      /home/${USERNAME}/.ccache \
      /home/${USERNAME}/.config \
      /home/${USERNAME}/.ignition \
      /home/${USERNAME}/.colcon \
      /home/${USERNAME}/.ros && \
    chown -R $USER_UID:$USER_GID /home/${USERNAME} /opt/overlay_ws/

# Install additional dependencies
# You can also add any necessary apt-get install, pip install, etc. commands at this point.
# NOTE: The /opt/overlay_ws folder contains MoveIt Pro binary packages and the source file.
# hadolint ignore=SC1091
ARG GZ_VERSION_ARG
ENV GZ_VERSION=${GZ_VERSION_ARG}
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    . /opt/overlay_ws/install/setup.sh && \
    apt-get update && \
    rosdep install -r -y \
    --from-paths ${USER_WS}/src

# Set up colcon defaults for the new user
USER ${USERNAME}
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
COPY ${ROBAST_PROJECT_STARTUP_RELATIVE_DIR}/colcon-defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml

# hadolint ignore=DL3002
USER root

###################################################################
# Target for the developer build which does not compile any code. #
###################################################################
FROM base as user-overlay-dev

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility

#install to use the nvidia gpu in the docker
RUN apt-get update && apt-get install -y \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

# Install any additional packages for development work
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        less \
        gdb \
        nano

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]

#########################################
# Target for compiled, deployable image #
#########################################
FROM base as user-overlay

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Compile the workspace
WORKDIR $USER_WS
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/home/${USERNAME}/.ccache \
    . /opt/overlay_ws/install/setup.sh && \
    colcon build

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]
