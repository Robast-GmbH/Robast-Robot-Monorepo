FROM python:3.10-slim

RUN apt-get update &&\
    apt-get upgrade -y &&\
    apt-get install -y \
    git\
    ffmpeg \
    libsm6 \
    libxext6

WORKDIR /map_cleaner

COPY ./requirements.txt ./

COPY ./src ./src