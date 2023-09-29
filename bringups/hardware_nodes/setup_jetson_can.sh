# In order to make this work run: sudo apt-get install can-utils

# Load CAN kernel module
sudo modprobe can
# Load CAN raw protocol kernel module
sudo modprobe can-raw
# Load SocketCAN (slcan) kernel module for CAN communication over serial line devices
sudo modprobe slcan

# Bring up the CAN network interface can0, setting the bitrate to 500,000 bps,
# data bitrate to 1,000,000 bps, enabling error reporting (berr-reporting), and enabling flexible data-rate (fd)
sudo ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on

#TODO: Find a proper way for the name of the container

# Retrieve the process ID of a Docker container named hardware_nodes_drawer
DOCKERPID=$(docker inspect -f '{{ .State.Pid }}' robo_a_hardware_nodes_1)
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