FROM ubuntu:focal
 
ENV DEBIAN_FRONTEND=noninteractive
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu && \
    rm -rf /var/lib/apt/lists/*
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENV USER ubuntu

COPY . /workspace/src

ENV DEBIAN_FRONTEND=
ENV DEBIAN_FRONTEND=noninteractive
RUN curl -sL https://deb.nodesource.com/setup_14.x | sudo -E bash -
RUN apt install nodejs
RUN apt-get update && apt upgrade -y && apt-get install net-tools 
RUN npm install @mui/material @emotion/react @emotion/styled && \
    npm install @mui/icons-material && \
    npm i --save react-select
ENV DEBIAN_FRONTEND=
# RUN npm install -g serve

 #SHELL ["/bin/bash", "-c"]
 #RUN cd workspace/src/; \
 #    npm run build \
 #    serve -s build&

WORKDIR workspace/src/
RUN npm install
CMD ["npm", "start"]  
EXPOSE 3000
