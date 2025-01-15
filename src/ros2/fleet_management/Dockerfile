ARG ROS_BUILD_DISTRO=humble
FROM ghcr.io/robast-gmbh/monorepo/robast-dev-core:${ROS_BUILD_DISTRO}
ARG ROS_BUILD_DISTRO

USER root
RUN pip3 install roslibpy pipenv nudged

WORKDIR /rmf-web-workspace
RUN git clone https://github.com/open-rmf/rmf-web.git
RUN chown -Rf robast .

WORKDIR /workspace

RUN echo "source /usr/share/bash-completion/completions/git" >> /home/${USER}/.bashrc \
&& echo "if [ -f /opt/ros/${ROS_BUILD_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USER}/.bashrc \
&& echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/${USER}/.bashrc

USER robast