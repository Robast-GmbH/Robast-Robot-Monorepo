# Check this link for further how to setip can on jetson board:
# https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html?#jetson-platform-details

### The following commands only need to be executed once to setup a new jetson board (compare above website) ###
#   sudo apt-get install busybox
# Pinmux for can0_din:
#   busybox devmem 0x0c303018 w 0xc458
# Pinmux for can0_dout:
#   busybox devmem 0x0c303010 w 0xc400


### The following commands need to be executed every time the jetson board is rebooted ###
# Load CAN kernel module
sudo modprobe can
# Load CAN raw protocol kernel module
sudo modprobe can-raw
# Load the MTT_CAN (Multi-threaded CAN) kernel module which adds real CAN interface support (for Jetson, mttcan)
sudo modprobe mttcan
# Bring up the CAN network interface can0, setting the can bitrate, data bitrate, enabling error reporting (berr-reporting), and enabling flexible data-rate (fd)
sudo ip link set can0 up type can bitrate 250000 dbitrate 1000000 berr-reporting on fd on
# Set the transmit queue length for can0 interface to 1000
sudo ifconfig can0 txqueuelen 1000