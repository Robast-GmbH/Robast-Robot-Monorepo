FROM python:3.10-slim as devel

RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast

#RUN echo 'export PATH=$PATH:/home/robast/.local/bin' >> /home/$USER/.bashrc
# Install dependencies
RUN apt-get update &&\
    apt-get upgrade -y &&\
    apt-get install -y \
    git\
    ffmpeg \
    libsm6 \
    libxext6 \
    libopencv-dev \
    file