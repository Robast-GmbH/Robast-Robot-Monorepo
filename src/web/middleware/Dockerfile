FROM python:3

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget git sudo && \
    rm -rf /var/lib/apt/lists/*
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN useradd --create-home --home-dir /home/robast --shell /bin/bash --user-group --groups adm,sudo robast && \
    echo robast:robast | chpasswd && \
    echo "robast ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER robast
ARG DEBIAN_FRONTEND=

WORKDIR /workspace

COPY main.py /workspace/middleware/main.py
COPY requirements.txt /workspace/middleware/requirements.txt

USER robast

#RUN pip install --no-cache-dir -r /workspace/middleware/requirements.txt
#CMD [ "python", "/workspace/middleware/main.py" ]
