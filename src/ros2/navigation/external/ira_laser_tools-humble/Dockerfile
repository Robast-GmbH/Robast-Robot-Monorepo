ARG ROS_BUILD_DISTRO=humble
FROM ros:${ROS_BUILD_DISTRO}-ros-base AS devel

ARG ROS_BUILD_DISTRO
WORKDIR /robast/src/
COPY . /robast/src/ira_laser_tools
WORKDIR /robast
ENV ROS_DISTRO=${ROS_BUILD_DISTRO}

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths /robast/src --ignore-src -r -y && \
    apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_BUILD_DISTRO}/setup.sh && \
    colcon build --install-base /robast/${ROS_BUILD_DISTRO}/ --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DBUILD_TESTING=OFF'

WORKDIR /robast/${ROS_BUILD_DISTRO}/
RUN echo "source /robast/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/robast/dds_configs/cyclone_DDS_config.xml

CMD ["bash", "-c", "source /robast/${ROS_DISTRO}/setup.bash && ros2 launch ira_laser_tools laserscan_multi_merger.launch.py"]

