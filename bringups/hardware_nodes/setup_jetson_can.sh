# Check this link for further how to setip can on jetson board:
# https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html?#jetson-platform-details

# Load CAN kernel module
sudo modprobe can
# Load CAN raw protocol kernel module
sudo modprobe can-raw
# Load the MTT_CAN (Multi-threaded CAN) kernel module which adds real CAN interface support (for Jetson, mttcan)
sudo modprobe mttcan
# Bring up the CAN network interface can0, setting the bitrate to 500,000 bps,
# data bitrate to 1,000,000 bps, enabling error reporting (berr-reporting), and enabling flexible data-rate (fd)
sudo ip link set can0 up type can bitrate 50000 dbitrate 100000 berr-reporting on fd on
# Set the transmit queue length for can0 interface to 1000
sudo ifconfig can0 txqueuelen 1000