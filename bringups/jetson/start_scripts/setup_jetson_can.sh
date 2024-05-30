#!/bin/sh
### In order to get CAN running on the jetson we need:
## (1) Make some initial setup commands once
## (2) Run some commands every time the jetson is started (which we automate using systemd)


### The following commands only need to be executed once to setup a new jetson board (compare below website) ###
### https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html?#jetson-platform-details ###
## Install busybox
#   sudo apt-get install busybox

### To let this script beeing executed at startup, we need to do the following once ###
## (1) Create a service using:
# sudo vim /etc/systemd/system/setup_can_bus.service
## (2) Fill that service file with this content (remove hashtags and check the correctness of the path to the script in ExecStart!!):
# [Unit]
# Description=Setup the can bus
# After=network.target
# [Service]
# ExecStart=/home/robast/Monorepo/bringups/jetson/start_scripts/setup_jetson_can.sh
# [Install]
# WantedBy=multi-user.target
## (3) Reload the systemd daemon to load the new service unit file
# sudo systemctl daemon-reload
## (4) Start the service
# sudo systemctl start setup_can_bus.service
## (5) Enable the service to start at boot
# sudo systemctl enable setup_can_bus.service
## (6) Optional: Check the status of the service
# sudo systemctl status setup_can_bus.service


### The following commands need to be executed every time the jetson board is rebooted ###
# Pinmux for can0_din:
sudo busybox devmem 0x0c303018 w 0xc458
# Pinmux for can0_dout:
sudo busybox devmem 0x0c303010 w 0xc400
# Load CAN kernel module
sudo modprobe can
# Load CAN raw protocol kernel module
sudo modprobe can-raw
# Load the MTT_CAN (Multi-threaded CAN) kernel module which adds real CAN interface support (for Jetson, mttcan)
sudo modprobe mttcan
# Bring up the CAN network interface can0, setting the can bitrate, data bitrate, enabling error reporting (berr-reporting), and enabling flexible data-rate (fd)
sudo ip link set can0 up type can bitrate 250000 dbitrate 1000000 berr-reporting on fd on
# Set the transmit queue length for can0 interface
# The value for this is choosen from the estimation that one requested led change of the base needs 129 messages (1 header + 128 states) and we want
# to make sure 4 base led changes can be queued and add a little bit of safety margin for possible other messages that might be sent in parellel
# TODO: Discuss the size of the txqueuelen
sudo ifconfig can0 txqueuelen 550