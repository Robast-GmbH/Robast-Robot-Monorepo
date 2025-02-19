FROM ros:humble-ros-core-jammy as release

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> ~root/.bashrc

RUN apt-get update && apt-get install -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  libboost-all-dev \
  ros-humble-ros-testing \
  ros-humble-ament-cmake-test \
  ros-humble-launch-testing \
  libboost-all-dev \
  git \
  && rosdep init || echo "rosdep already initialized"

WORKDIR /ws
RUN mkdir src; cd src; git clone https://github.com/SICKAG/sick_safevisionary_ros2.git; \
  git clone https://github.com/SICKAG/sick_safevisionary_base.git && \
  rosdep init; rosdep update

RUN rosdep install --from-paths ./ --ignore-src -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh; colcon build --packages-select sick_safevisionary_base sick_safevisionary_interfaces sick_safevisionary_driver  --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi" >> ~root/.bashrc

ENV ROS_DOMAIN_ID=0
# dds setup / configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/ws/dds_configs/fast_DDS_config.xml
ENV CYCLONEDDS_URI=/ws/dds_configs/cyclone_DDS_config.xml