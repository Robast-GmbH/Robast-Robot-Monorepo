FROM ros:foxy-ros-core-focal

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep 
  
#nfc stuff
RUN apt-get install --no-install-recommends -y \
    libnfc-bin \
    pcscd \ 
    libpcsclite1 \ 
    libusb-1.0-0 \ 
    flex \
    perl \
    pkg-config \
    libpcsclite-dev \
    libusb-1.0-0-dev \
    usbutils

ADD acsccid-1.1.8.tar.bz2 /workspaces
#ADD https://www.acs.com.hk/download-driver-unified/12030/ACS-Unified-Driver-Lnx-Mac-118-P.zip  /workspaces
RUN  cd /workspaces/acsccid-1.1.8/ \
    ./configure \
    ./configure --disable-dependency-tracking \
    make \
    make install \
