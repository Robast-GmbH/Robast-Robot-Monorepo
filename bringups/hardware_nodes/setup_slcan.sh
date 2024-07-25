# Load SocketCAN (slcan) kernel module for CAN communication over serial line devices
sudo modprobe slcan

# Attach SocketCAN device to a serial line (replace /dev/robast/robast_can with the appropriate serial line)
sudo slcan_attach -f -s5 -o /dev/serial/by-id/usb-Microchip_Technology__Inc._USBtin_A0211324-if00
# Create a CAN network device named can1 using the specified serial line device
sudo slcand serial/by-id/usb-Microchip_Technology__Inc._USBtin_A0211324-if00 can1
# Bring the CAN network interface can1 up
sudo ifconfig can1 up
# Set the transmit queue length for can1 interface
# The value for this is choosen from the estimation that one requested led change of the base needs 129 messages (1 header + 128 states) and we want
# to make sure 4 base led changes can be queued and add a little bit of safety margin for possible other messages that might be sent in parellel
# TODO: Discuss the size of the txqueuelen
sudo ifconfig can1 txqueuelen 550

