# In order to make this work run: sudo apt-get install can-utils

# Load CAN kernel module
sudo modprobe can
# Load CAN raw protocol kernel module
sudo modprobe can-raw
# Load SocketCAN (slcan) kernel module for CAN communication over serial line devices
sudo modprobe slcan

# Attach SocketCAN device to a serial line (replace /dev/robast/robast_can with the appropriate serial line)
sudo slcan_attach -f -s5 -o /dev/robast/robast_can
# Create a CAN network device named can0 using the specified serial line device
sudo slcand robast/robast_can can0
# Bring the CAN network interface can0 up
sudo ifconfig can0 up
# Set the transmit queue length for can0 interface
# The value for this is choosen from the estimation that one requested led change of the base needs 129 messages (1 header + 128 states) and we want
# to make sure 4 base led changes can be queued and add a little bit of safety margin for possible other messages that might be sent in parellel
# TODO: Discuss the size of the txqueuelen
sudo ifconfig can0 txqueuelen 550

# Retrieve the process ID of a Docker container named hardware_nodes_drawer
DOCKERPID=$(docker inspect -f '{{ .State.Pid }}' hardware_nodes_drawer)
# Create a pair of virtual CAN network interfaces (vxcan0 and vxcan1) in the specified network namespace associated with the Docker container
sudo ip link add vxcan0 type vxcan peer name vxcan1 netns $DOCKERPID
# Load CAN gateway kernel module
sudo modprobe can-gw
# Configure CAN gateway to forward CAN messages from can0 to vxcan0
sudo cangw -A -s can0 -d vxcan0 -e
# Configure CAN gateway to forward CAN messages from vxcan0 to can0
sudo cangw -A -s vxcan0 -d can0 -e
# Bring the virtual CAN network interface vxcan0 up
sudo ip link set vxcan0 up
# Bring the CAN network interface can0 up
sudo ip link set can0 up
# Execute 'ip link set vxcan1 up' inside the Docker container's network namespace, bringing the virtual CAN network interface vxcan1 up
sudo nsenter -t $DOCKERPID -n ip link set vxcan1 up